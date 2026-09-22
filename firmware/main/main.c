/*
 * main.c - Inicializacao do carrinho.
 *
 * Cria as cinco tarefas da Tabela 6. A ordem importa: o controle entra antes
 * de tudo para deixar a ponte H em estado neutro (RF10), e so depois sobem os
 * sensores, o comando e a rede.
 */
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
    ESP_LOGI(TAG, "carrinho %s iniciando", CARRINHO_ID);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Servico de interrupcao compartilhado pelos ecos do sonar e pelos
     * encoders. Instalado uma vez so, antes de qualquer gpio_isr_handler_add. */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    state_init();

    control_start();     /* primeiro: ponte H neutra antes de qualquer comando */
    sonar_start();
    odometry_start();    /* calibra o giroscopio; o carrinho precisa estar parado */
    bt_command_start();
    telemetry_start();

    ESP_LOGI(TAG, "todas as tarefas ativas");
}
