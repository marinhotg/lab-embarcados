/* The only task that drives the H-bridge. Precedence: a command timeout zeroes
 * everything; an invalid front reading or an obstacle closer than
 * OBSTACLE_BRAKE_M blocks forward motion but keeps rotation, so the rover can
 * still turn away. */
#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "state.h"
#include "tasks.h"

static const char *TAG = "control";

#define CH_LEFT   LEDC_CHANNEL_0
#define CH_RIGHT  LEDC_CHANNEL_1

static TaskHandle_t s_task;

void control_notify(void)
{
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
}

static void drive_channel(int pin_a, int pin_b, ledc_channel_t channel,
                          int dir, float duty)
{
    gpio_set_level(pin_a, dir > 0 ? 1 : 0);
    gpio_set_level(pin_b, dir < 0 ? 1 : 0);

    uint32_t raw = (uint32_t)(duty * PWM_DUTY_MAX_RAW);
    if (raw > PWM_DUTY_MAX_RAW) {
        raw = PWM_DUTY_MAX_RAW;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, raw);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

/* Maps onto [MIN, MAX]: below MOTOR_DUTY_MIN the motor hums without turning. */
static void speed_to_drive(float v_mps, int *dir, float *duty)
{
    float magnitude = fabsf(v_mps) / V_MAX_MPS;

    if (magnitude < 0.01f) {
        *dir  = 0;
        *duty = 0.0f;
        return;
    }

    if (magnitude > 1.0f) {
        magnitude = 1.0f;
    }

    *dir  = (v_mps > 0.0f) ? 1 : -1;
    *duty = MOTOR_DUTY_MIN + magnitude * (MOTOR_DUTY_MAX - MOTOR_DUTY_MIN);
}

static void control_task(void *arg)
{
    (void)arg;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PERIOD_CONTROL_MS));

        rover_state_t snap;
        state_snapshot(&snap);

        float v = snap.cmd.v_ref_mps;
        float w = snap.cmd.w_ref_rad_s;

        bool timeout      = snap.safety.timeout_active;
        bool front_stale  = SAFETY_REQUIRE_FRONT_SONAR ? true : false;
        bool auto_brake   = false;

        int64_t now_us = esp_timer_get_time();
        const sonar_state_t *front = &snap.sonar[SONAR_FRONT];

        if (front->valid) {
            int64_t age_ms = (now_us - front->stamp_us) / 1000;
            front_stale = (age_ms > SONAR_MAX_AGE_MS);

            /* Raw value, not the median: braking cannot wait for three readings. */
            if (!front_stale && front->raw_m < OBSTACLE_BRAKE_M) {
                auto_brake = true;
            }
        }

        if (timeout) {
            v = 0.0f;
            w = 0.0f;
        } else if (v > 0.0f && (front_stale || auto_brake)) {
            v = 0.0f;
        }

        float v_left  = v - WHEEL_BASE_M * w / 2.0f;
        float v_right = v + WHEEL_BASE_M * w / 2.0f;

        /* Scale both sides together; clipping only one would distort the curve. */
        float peak = fmaxf(fabsf(v_left), fabsf(v_right));
        if (peak > V_MAX_MPS) {
            float scale = V_MAX_MPS / peak;
            v_left  *= scale;
            v_right *= scale;
        }

        int   dir_left, dir_right;
        float duty_left, duty_right;
        speed_to_drive(v_left,  &dir_left,  &duty_left);
        speed_to_drive(v_right, &dir_right, &duty_right);

        drive_channel(PIN_DIR_LEFT_A,  PIN_DIR_LEFT_B,  CH_LEFT,  dir_left,  duty_left);
        drive_channel(PIN_DIR_RIGHT_A, PIN_DIR_RIGHT_B, CH_RIGHT, dir_right, duty_right);

        rover_state_t *st = state_lock();
        st->drive.dir_left      = dir_left;
        st->drive.dir_right     = dir_right;
        st->drive.duty_left     = duty_left;
        st->drive.duty_right    = duty_right;
        st->safety.auto_brake   = auto_brake;
        st->safety.front_invalid = front_stale;
        state_unlock();
    }
}

void control_start(void)
{
    gpio_config_t dir_cfg = {
        .pin_bit_mask = (1ULL << PIN_DIR_LEFT_A)  | (1ULL << PIN_DIR_LEFT_B) |
                        (1ULL << PIN_DIR_RIGHT_A) | (1ULL << PIN_DIR_RIGHT_B),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dir_cfg));

    gpio_set_level(PIN_DIR_LEFT_A,  0);
    gpio_set_level(PIN_DIR_LEFT_B,  0);
    gpio_set_level(PIN_DIR_RIGHT_A, 0);
    gpio_set_level(PIN_DIR_RIGHT_B, 0);

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    const int pwm_pins[2]           = { PIN_PWM_LEFT, PIN_PWM_RIGHT };
    const ledc_channel_t channels[2] = { CH_LEFT, CH_RIGHT };

    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = pwm_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = channels[i],
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }

#if !SAFETY_REQUIRE_FRONT_SONAR
    ESP_LOGW(TAG, "BENCH MODE: forward motion allowed without front sonar");
#endif
    ESP_LOGI(TAG, "H-bridge neutral");

    xTaskCreate(control_task, "control", 4096, NULL, PRIO_CONTROL, &s_task);
}
