/* Tarefa Controle, unica que escreve na ponte H. Roda a cada 50 ms e e acordada
 * fora do periodo pela parada prioritaria (RF03) e pelo timeout (RF08).
 * Precedencia: timeout zera tudo; leitura frontal invalida ou obstaculo abaixo
 * de 30 cm impedem o avanco mas preservam a rotacao. */
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

/* dir: -1 re, 0 parado, +1 avanco. duty normalizado em 0..1. */
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

/* Remapeia para [MIN, MAX]: abaixo de MOTOR_DUTY_MIN o motor zumbe sem girar. */
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
        /* Acorda no periodo ou no evento, o que vier primeiro. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PERIOD_CONTROL_MS));

        carrinho_state_t snap;
        state_snapshot(&snap);

        float v = snap.cmd.v_ref_mps;
        float w = snap.cmd.w_ref_rad_s;

        bool timeout      = snap.safety.timeout_active;
        bool front_stale  = true;
        bool auto_brake   = false;

        int64_t now_us = esp_timer_get_time();
        const sonar_state_t *front = &snap.sonar[SONAR_FRENTE];

        if (front->valid) {
            int64_t age_ms = (now_us - front->stamp_us) / 1000;
            front_stale = (age_ms > SONAR_MAX_AGE_MS);

            /* Valor bruto, nao a mediana: a seguranca nao espera (secao 3.2). */
            if (!front_stale && front->raw_m < OBSTACLE_BRAKE_M) {
                auto_brake = true;
            }
        }

        if (timeout) {
            v = 0.0f;
            w = 0.0f;
        } else if (v > 0.0f && (front_stale || auto_brake)) {
            /* Preserva a rotacao: da para girar e sair de frente do obstaculo. */
            v = 0.0f;
        }

        /* Cinematica inversa da secao 4.6. */
        float v_left  = v - WHEEL_BASE_M * w / 2.0f;
        float v_right = v + WHEEL_BASE_M * w / 2.0f;

        /* Reduz os dois lados na mesma proporcao; encolher so um distorceria a curva. */
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

        carrinho_state_t *st = state_lock();
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
    /* RF10: sentido em nivel baixo e duty zero, antes de qualquer comando. */
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

    ESP_LOGI(TAG, "ponte H em estado neutro");

    xTaskCreate(control_task, "control", 4096, NULL, PRIO_CONTROL, &s_task);
}
