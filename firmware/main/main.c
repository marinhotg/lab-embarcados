#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "config.h"
#include "state.h"
#include "tasks.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "rover %s starting", ROVER_ID);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Shared by the sonar echoes and the encoders; must precede any
     * gpio_isr_handler_add. */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    state_init();

    control_start();     /* first, so the H-bridge is neutral before anything else */
    sonar_start();
    odometry_start();    /* calibrates the gyro: the rover must be still */
    bt_command_start();
    telemetry_start();

    ESP_LOGI(TAG, "all tasks running");
}
