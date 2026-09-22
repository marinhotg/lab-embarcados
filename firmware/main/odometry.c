/*
 * odometry.c - Tarefa Odometria e IMU (Tabela 6, 20 ms).
 *
 * Porte corrigido de calculate_esp.cpp do projeto anterior (PCS3848), que
 * implementava as mesmas equacoes (1) a (5) da secao 4.5. Tres defeitos do
 * original foram corrigidos aqui:
 *
 *   1. delta_theta usava /(2*b) em vez de /b, subestimando a rotacao pela
 *      metade. A equacao (2) do documento e Dtheta = (Ds_R - Ds_L)/b.
 *   2. Pulsos por volta estavam embutidos como M_PI/15 na ESP e np.pi/20 no
 *      cliente Python. Agora e ENCODER_PULSES_PER_REV, um valor so.
 *   3. Graus e radianos se misturavam. Aqui tudo e radiano; a conversao, se
 *      houver, acontece na borda.
 *
 * Alem disso, os pulsos sao contados por interrupcao (RF05) e nao por
 * amostragem de digitalRead no laco principal, que perdia bordas.
 */
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

/* Contadores monotonicos, nunca zerados. A ISR e o unico escritor e a tarefa
 * o unico leitor, entao a diferenca entre duas amostragens e exata sem regiao
 * critica: incremento de 32 bits alinhado nao se parte, e uma leitura atrasada
 * so adia o pulso para o ciclo seguinte. */
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

/* Nucleo cinematico, sem estado global e sem dependencia de hardware, para
 * poder ser conferido contra a simulacao Python fora da placa. */
typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;
    float yaw_odom_rad;
    float ds_m;        /* deslocamento do ultimo passo, saida */
} odom_pose_t;

/* Nao e static: os testes de host em firmware/test a exercitam diretamente. */
void odom_step(odom_pose_t *p, float ds_left, float ds_right,
               float gyro_z_rad_s, float dt, bool imu_valid);

void odom_step(odom_pose_t *p, float ds_left, float ds_right,
               float gyro_z_rad_s, float dt, bool imu_valid)
{
    /* Equacao (2). O projeto anterior dividia por (2*b) aqui e subestimava a
     * rotacao pela metade; o documento define Dtheta = (Ds_R - Ds_L)/b. */
    float ds     = (ds_right + ds_left) / 2.0f;
    float dtheta = (ds_right - ds_left) / WHEEL_BASE_M;

    /* Equacao (3), so por encoder, mantido separado. */
    float yaw_odom = wrap_pi(p->yaw_odom_rad + dtheta);

    /* Equacao (6), filtro complementar, escrito de forma segura na passagem
     * por +-pi: em vez de misturar dois angulos absolutos, corrige a predicao
     * do giroscopio pelo erro angular envolvido. */
    float yaw_prev = p->yaw_rad;
    float yaw_fused;

    if (imu_valid) {
        float yaw_pred = yaw_prev + gyro_z_rad_s * dt;
        float error    = wrap_pi(yaw_odom - yaw_pred);
        yaw_fused      = wrap_pi(yaw_pred + (1.0f - COMPLEMENTARY_ALPHA) * error);
    } else {
        yaw_fused = yaw_odom;
    }

    /* Equacoes (4) e (5): projeta Ds no ponto medio do arco, usando o yaw
     * fundido, conforme a secao 4.5. */
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

    /* Distancia percorrida por pulso: 2*pi*r/N, da equacao (1). */
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

        uint32_t now_left  = s_pulses_left;
        uint32_t now_right = s_pulses_right;
        uint32_t pulses_left  = now_left  - last_left;    /* estouro de 32 bits
                                                             se resolve sozinho */
        uint32_t pulses_right = now_right - last_right;
        last_left  = now_left;
        last_right = now_right;

        float accel[3] = { 0 };
        float gyro[3]  = { 0 };
        bool  imu_valid = s_imu_ok && (mpu6050_read(accel, gyro) == ESP_OK);

        carrinho_state_t *st = state_lock();

        /* O encoder optico tem um canal so e nao mede sentido. O sinal vem do
         * acionamento comandado de cada lado, como admite a secao 4.5. Com a
         * ponte H desligada os pulsos sao descartados: sao inercia ou empurrao,
         * e associa-los a um sentido so introduziria erro. */
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
    gpio_config_t enc_cfg = {
        .pin_bit_mask = (1ULL << PIN_ENCODER_LEFT) | (1ULL << PIN_ENCODER_RIGHT),
        .mode         = GPIO_MODE_INPUT,
        /* GPIO 34/35 sao input-only e nao tem pull-up interno. A polarizacao
         * externa em 3,3 V e obrigatoria (RNF06); as linhas abaixo nao tem
         * efeito nesses pinos e estao aqui so para deixar a intencao clara. */
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&enc_cfg));

    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENCODER_LEFT,  encoder_left_isr,  NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENCODER_RIGHT, encoder_right_isr, NULL));

    s_imu_ok = (mpu6050_init() == ESP_OK);
    if (s_imu_ok) {
        ESP_LOGI(TAG, "calibrando giroscopio - manter o carrinho parado");
        mpu6050_calibrate_gyro();
    } else {
        ESP_LOGW(TAG, "IMU ausente; yaw ficara so por odometria");
    }

    xTaskCreate(odometry_task, "odometry", 4096, NULL, PRIO_ODOMETRY, NULL);
}
