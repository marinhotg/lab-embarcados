/* Differential-drive odometry fused with the MPU6050 gyro. Everything is in
 * radians; encoder pulses are counted by the PCNT peripheral. */
#include <math.h>

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "mpu6050.h"
#include "state.h"
#include "tasks.h"

static const char *TAG = "odometry";

/* Hardware counters with a glitch filter: the motor PWM couples microsecond
 * spikes into the encoder wires, which a GPIO interrupt counted as slots and
 * turned into metres of phantom travel. A real slot lasts milliseconds. The
 * counts accumulate past the 16-bit limit and are never cleared, so the
 * difference between two samples is exact. */
static pcnt_unit_handle_t s_encoder_left;
static pcnt_unit_handle_t s_encoder_right;

static bool s_imu_ok;

static pcnt_unit_handle_t encoder_unit(int gpio)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit  = -1,
        .high_limit = ENCODER_PCNT_LIMIT,
        .flags.accum_count = 1,
    };
    pcnt_unit_handle_t unit;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = ENCODER_GLITCH_NS };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));

    pcnt_chan_config_t chan_cfg = { .edge_gpio_num = gpio, .level_gpio_num = -1 };
    pcnt_channel_handle_t chan;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_cfg, &chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    /* accum_count needs a watch point at the limit to carry the overflow. */
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, ENCODER_PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit;
}

static uint32_t encoder_count(pcnt_unit_handle_t unit)
{
    int count = 0;
    pcnt_unit_get_count(unit, &count);
    return (uint32_t)count;
}

static float wrap_pi(float angle)
{
    while (angle > (float)M_PI)  { angle -= 2.0f * (float)M_PI; }
    while (angle < -(float)M_PI) { angle += 2.0f * (float)M_PI; }
    return angle;
}

/* Kinematic core with no globals or hardware, so it can be tested on the host. */
typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;
    float yaw_odom_rad;
    float ds_m;        /* output: distance travelled in the last step */
} odom_pose_t;

void odom_step(odom_pose_t *p, float ds_left, float ds_right,
               float gyro_z_rad_s, float dt, bool imu_valid);

void odom_step(odom_pose_t *p, float ds_left, float ds_right,
               float gyro_z_rad_s, float dt, bool imu_valid)
{
    float ds     = (ds_right + ds_left) / 2.0f;
    float dtheta = (ds_right - ds_left) / WHEEL_BASE_M;

    float yaw_odom = wrap_pi(p->yaw_odom_rad + dtheta);

    /* Complementary filter applied to the wrapped error instead of blending two
     * absolute angles, so it stays correct across +-pi. */
    float yaw_prev = p->yaw_rad;
    float yaw_fused;

    if (imu_valid) {
        float yaw_pred = yaw_prev + gyro_z_rad_s * dt;
        float error    = wrap_pi(yaw_odom - yaw_pred);
        yaw_fused      = wrap_pi(yaw_pred + (1.0f - COMPLEMENTARY_ALPHA) * error);
    } else {
        yaw_fused = yaw_odom;
    }

    /* Project the step on the heading at the middle of the arc. */
    float heading_mid = yaw_prev + wrap_pi(yaw_fused - yaw_prev) / 2.0f;

    p->x_m         += ds * cosf(heading_mid);
    p->y_m         += ds * sinf(heading_mid);
    p->yaw_rad      = yaw_fused;
    p->yaw_odom_rad = yaw_odom;
    p->ds_m         = ds;
}

static void odometry_task(void *arg)
{
    (void)arg;

    const float meters_per_pulse =
        2.0f * (float)M_PI * WHEEL_RADIUS_M / ENCODER_PULSES_PER_REV;

    TickType_t next = xTaskGetTickCount();
    int64_t  last_us    = esp_timer_get_time();
    uint32_t last_left  = encoder_count(s_encoder_left);
    uint32_t last_right = encoder_count(s_encoder_right);

    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(PERIOD_ODOMETRY_MS));

        int64_t now_us = esp_timer_get_time();
        float   dt     = (now_us - last_us) / 1e6f;
        last_us = now_us;
        if (dt <= 0.0f) {
            continue;
        }

        /* Unsigned subtraction handles counter wrap-around. */
        uint32_t now_left  = encoder_count(s_encoder_left);
        uint32_t now_right = encoder_count(s_encoder_right);
        uint32_t pulses_left  = now_left  - last_left;
        uint32_t pulses_right = now_right - last_right;
        last_left  = now_left;
        last_right = now_right;

        float accel[3] = { 0 };
        float gyro[3]  = { 0 };
        bool  imu_valid = s_imu_ok && (mpu6050_read(accel, gyro) == ESP_OK);

        rover_state_t *st = state_lock();

        /* Single-channel optical encoders cannot sense direction, so the sign
         * comes from the commanded drive. Pulses with the bridge off come from
         * inertia or pushing and are discarded rather than guessed. */
        int dir_left  = st->drive.dir_left;
        int dir_right = st->drive.dir_right;

        float ds_left  = (dir_left  == 0) ? 0.0f
                                          : dir_left  * (float)pulses_left  * meters_per_pulse;
        float ds_right = (dir_right == 0) ? 0.0f
                                          : dir_right * (float)pulses_right * meters_per_pulse;

        odom_pose_t p = {
            .x_m          = st->pose.x_m,
            .y_m          = st->pose.y_m,
            .yaw_rad      = st->pose.yaw_rad,
            .yaw_odom_rad = st->pose.yaw_odom_rad,
        };
        odom_step(&p, ds_left, ds_right, gyro[2], dt, imu_valid);

        st->pose.x_m          = p.x_m;
        st->pose.y_m          = p.y_m;
        st->pose.yaw_rad      = p.yaw_rad;
        st->pose.yaw_odom_rad = p.yaw_odom_rad;

        st->wheel.v_left_mps   = ds_left  / dt;
        st->wheel.v_right_mps  = ds_right / dt;
        st->wheel.v_linear_mps = p.ds_m / dt;

        if (imu_valid) {
            for (int a = 0; a < 3; a++) {
                st->imu.accel_m_s2[a] = accel[a];
                st->imu.gyro_rad_s[a] = gyro[a];
            }
            st->imu.stamp_us = now_us;
        }
        st->imu.valid = imu_valid;

        state_unlock();
    }
}

void odometry_start(void)
{
    /* GPIO 34/35 have no internal pulls: the external pull-up is required. */
    s_encoder_left  = encoder_unit(PIN_ENCODER_LEFT);
    s_encoder_right = encoder_unit(PIN_ENCODER_RIGHT);

    s_imu_ok = (mpu6050_init() == ESP_OK);
    if (s_imu_ok) {
        ESP_LOGI(TAG, "calibrating gyro, keep the rover still");
        mpu6050_calibrate_gyro();
    } else {
        ESP_LOGW(TAG, "IMU not found, yaw from encoders only");
    }

    xTaskCreate(odometry_task, "odometry", 4096, NULL, PRIO_ODOMETRY, NULL);
}
