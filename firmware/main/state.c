#include "state.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static carrinho_state_t  s_state;
static SemaphoreHandle_t s_mutex;

void state_init(void)
{
    s_mutex = xSemaphoreCreateMutex();

    /* RF10: atuadores neutros na inicializacao. */
    s_state.drive.dir_left  = 0;
    s_state.drive.dir_right = 0;
    s_state.cmd.last_packet_us = 0;
    s_state.safety.timeout_active = true;
}

void state_snapshot(carrinho_state_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_state;
    xSemaphoreGive(s_mutex);
}

carrinho_state_t *state_lock(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    return &s_state;
}

void state_unlock(void)
{
    xSemaphoreGive(s_mutex);
}

int64_t state_uptime_ms(void)
{
    return esp_timer_get_time() / 1000;
}
