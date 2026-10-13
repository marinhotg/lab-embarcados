/* Drives forward for a fixed time and stops. No radios, sensors or tasks.
 *
 * Powered from the H-bridge 5 V pin, the ESP boots when the supply turns on, so
 * the supply switch is the trigger and USB can stay unplugged. */
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"

#define DUTY_PCT   80
#define DRIVE_S    10
#define DELAY_S     5   /* time to put the rover down after power-on */

static const char *TAG = "load_test";

static void set_motors(int on)
{
    /* If the rover spins instead of going straight, swap PIN_DIR_RIGHT_A and
     * PIN_DIR_RIGHT_B in the two channel B lines. */
    gpio_set_level(PIN_DIR_LEFT_A,  on);
    gpio_set_level(PIN_DIR_LEFT_B,  0);
    gpio_set_level(PIN_DIR_RIGHT_A, on);
    gpio_set_level(PIN_DIR_RIGHT_B, 0);

    uint32_t duty = on ? (PWM_DUTY_MAX_RAW * DUTY_PCT) / 100 : 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void app_main(void)
{
    gpio_config_t dir = {
        .pin_bit_mask = (1ULL << PIN_DIR_LEFT_A)  | (1ULL << PIN_DIR_LEFT_B) |
                        (1ULL << PIN_DIR_RIGHT_A) | (1ULL << PIN_DIR_RIGHT_B),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dir));

    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    const int pwm[2] = { PIN_PWM_LEFT, PIN_PWM_RIGHT };
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = pwm[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = LEDC_CHANNEL_0 + i,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }

    set_motors(0);

    for (int s = DELAY_S; s > 0; s--) {
        ESP_LOGI(TAG, "starting in %d...", s);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGW(TAG, "DRIVING %d s at %d%% duty", DRIVE_S, DUTY_PCT);
    set_motors(1);
    for (int s = DRIVE_S; s > 0; s--) {
        ESP_LOGI(TAG, "%d", s);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    set_motors(0);
    ESP_LOGW(TAG, "STOPPED. Power-cycle the supply to repeat.");
}
