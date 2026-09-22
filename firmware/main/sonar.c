/*
 * sonar.c - Tarefa Sonar (Tabela 6, 100 ms).
 *
 * Dispara um HC-SR04 por ativacao, alternando frente, tras, esquerda e
 * direita; a varredura completa leva 400 ms e nunca ha disparos simultaneos
 * (RF04). O eco e medido por interrupcao de borda com esp_timer, nao por
 * espera ocupada - a tarefa fica bloqueada num semaforo enquanto o pulso
 * viaja.
 *
 * Duas saidas por sensor, de proposito:
 *   dist_m  mediana das ultimas tres leituras validas, usada na telemetria
 *   raw_m   ultima leitura bruta valida, usada pela frenagem automatica
 *
 * A secao 3.2 do documento pede exatamente essa separacao: a seguranca nao
 * pode esperar a mediana se ha um obstaculo a 30 cm.
 */
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "config.h"
#include "state.h"
#include "tasks.h"

static const char *TAG = "sonar";

static const int s_trigger_pins[SONAR_COUNT] = PIN_SONAR_TRIGGERS;
static const int s_echo_pins[SONAR_COUNT]    = PIN_SONAR_ECHOS;

static const char *s_names[SONAR_COUNT] = { "frente", "tras", "esquerda", "direita" };

static volatile int64_t s_rise_us[SONAR_COUNT];
static volatile int64_t s_fall_us[SONAR_COUNT];
static SemaphoreHandle_t s_echo_done[SONAR_COUNT];

/* Janela da mediana, por sensor. */
static float s_window[SONAR_COUNT][SONAR_MEDIAN_WINDOW];
static int   s_window_len[SONAR_COUNT];
static int   s_window_pos[SONAR_COUNT];

static void IRAM_ATTR echo_isr(void *arg)
{
    int idx = (int)(intptr_t)arg;

    if (gpio_get_level(s_echo_pins[idx])) {
        s_rise_us[idx] = esp_timer_get_time();
    } else {
        s_fall_us[idx] = esp_timer_get_time();
        BaseType_t hp_woken = pdFALSE;
        xSemaphoreGiveFromISR(s_echo_done[idx], &hp_woken);
        if (hp_woken) {
            portYIELD_FROM_ISR();
        }
    }
}

static float median_of(const float *values, int len)
{
    if (len == 1) {
        return values[0];
    }

    float sorted[SONAR_MEDIAN_WINDOW];
    memcpy(sorted, values, len * sizeof(float));

    for (int i = 1; i < len; i++) {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    return sorted[len / 2];
}

static void push_sample(int idx, float dist_m)
{
    s_window[idx][s_window_pos[idx]] = dist_m;
    s_window_pos[idx] = (s_window_pos[idx] + 1) % SONAR_MEDIAN_WINDOW;
    if (s_window_len[idx] < SONAR_MEDIAN_WINDOW) {
        s_window_len[idx]++;
    }
}

/* Dispara um sensor e devolve a distancia em metros, ou -1 se nao houve eco
 * valido. */
static float measure(int idx)
{
    xSemaphoreTake(s_echo_done[idx], 0);   /* limpa eco antigo */
    s_rise_us[idx] = 0;
    s_fall_us[idx] = 0;

    gpio_intr_enable(s_echo_pins[idx]);

    /* Pulso de trigger de 10 us. */
    gpio_set_level(s_trigger_pins[idx], 1);
    esp_rom_delay_us(10);
    gpio_set_level(s_trigger_pins[idx], 0);

    BaseType_t got = xSemaphoreTake(s_echo_done[idx],
                                    pdMS_TO_TICKS(SONAR_ECHO_TIMEOUT_MS));
    gpio_intr_disable(s_echo_pins[idx]);

    if (got != pdTRUE || s_rise_us[idx] == 0) {
        return -1.0f;   /* ausencia de eco */
    }

    int64_t width_us = s_fall_us[idx] - s_rise_us[idx];
    if (width_us <= 0) {
        return -1.0f;
    }

    /* ida e volta: divide por dois */
    float dist_m = (float)width_us * SOUND_SPEED_M_S / 2.0f / 1e6f;

    if (dist_m < SONAR_MIN_VALID_M || dist_m > SONAR_MAX_VALID_M) {
        return -1.0f;   /* fora da faixa do modulo */
    }

    return dist_m;
}

static void sonar_task(void *arg)
{
    (void)arg;

    int idx = 0;
    TickType_t next = xTaskGetTickCount();

    for (;;) {
        float dist_m = measure(idx);

        if (dist_m > 0.0f) {
            push_sample(idx, dist_m);

            carrinho_state_t *st = state_lock();
            st->sonar[idx].raw_m    = dist_m;
            st->sonar[idx].dist_m   = median_of(s_window[idx], s_window_len[idx]);
            st->sonar[idx].stamp_us = esp_timer_get_time();
            st->sonar[idx].valid    = true;
            state_unlock();
        } else {
            /* Leitura rejeitada: mantem a ultima valida e deixa a idade
             * crescer. E a idade que o cliente e a seguranca consultam. */
            ESP_LOGD(TAG, "%s sem eco valido", s_names[idx]);
        }

        idx = (idx + 1) % SONAR_COUNT;
        vTaskDelayUntil(&next, pdMS_TO_TICKS(PERIOD_SONAR_MS));
    }
}

void sonar_start(void)
{
    uint64_t trigger_mask = 0;
    uint64_t echo_mask    = 0;

    for (int i = 0; i < SONAR_COUNT; i++) {
        trigger_mask |= 1ULL << s_trigger_pins[i];
        echo_mask    |= 1ULL << s_echo_pins[i];
    }

    gpio_config_t trig_cfg = {
        .pin_bit_mask = trigger_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trig_cfg));

    gpio_config_t echo_cfg = {
        .pin_bit_mask = echo_mask,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo_cfg));

    for (int i = 0; i < SONAR_COUNT; i++) {
        gpio_set_level(s_trigger_pins[i], 0);
        s_echo_done[i] = xSemaphoreCreateBinary();
        ESP_ERROR_CHECK(gpio_isr_handler_add(s_echo_pins[i], echo_isr,
                                             (void *)(intptr_t)i));
        /* So o sensor em medicao fica com interrupcao habilitada. */
        gpio_intr_disable(s_echo_pins[i]);
    }

    xTaskCreate(sonar_task, "sonar", 3072, NULL, PRIO_SONAR, NULL);
}
