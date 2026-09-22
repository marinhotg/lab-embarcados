/* Differential-drive odometry fused with the MPU6050 gyro. Everything is in
 * radians; encoder pulses are counted by interrupt. */
#include <math.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "mpu6050.h"
#include "state.h"
#include "tasks.h"

static const char *TAG = "odometry";

/* Monotonic counters, never reset. The ISR is the only writer and the task the
 * only reader, so the difference between two samples is exact without a
 * critical section: an aligned 32-bit increment is atomic, and a late read only
 * defers the pulse to the next cycle. */
static volatile uint32_t s_pulses_left;
static volatile uint32_t s_pulses_right;

static bool s_imu_ok;

static void IRAM_ATTR encoder_left_isr(void *arg)
{
    (void)arg;
    s_pulses_left++;
}

static void IRAM_ATTR encoder_right_isr(void *arg)
{
    (void)arg;
    s_pulses_right++;
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
    uint32_t last_left  = s_pulses_left;
    uint32_t last_right = s_pulses_right;

    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(PERIOD_ODOMETRY_MS));

        int64_t now_us = esp_timer_get_time();
        float   dt     = (now_us - last_us) / 1e6f;
        last_us = now_us;
        if (dt <= 0.0f) {
            continue;
        }

        /* Unsigned subtraction handles counter wrap-around. */
        uint32_t now_left  = s_pulses_left;
        uint32_t now_right = s_pulses_right;
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
    /* GPIO 34/35 have no internal pulls; these flags are no-ops, the external
     * pull-up is required. */
    gpio_config_t enc_cfg = {
        .pin_bit_mask = (1ULL << PIN_ENCODER_LEFT) | (1ULL << PIN_ENCODER_RIGHT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&enc_cfg));

    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENCODER_LEFT,  encoder_left_isr,  NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENCODER_RIGHT, encoder_right_isr, NULL));

    s_imu_ok = (mpu6050_init() == ESP_OK);
    if (s_imu_ok) {
        ESP_LOGI(TAG, "calibrating gyro, keep the rover still");
        mpu6050_calibrate_gyro();
    } else {
        ESP_LOGW(TAG, "IMU not found, yaw from encoders only");
    }

    xTaskCreate(odometry_task, "odometry", 4096, NULL, PRIO_ODOMETRY, NULL);
}
