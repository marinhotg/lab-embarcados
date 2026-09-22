/*
 * telemetry.c - Tarefa MQTT (Tabela 6, 200 ms).
 *
 * Wi-Fi em modo estacao, publicacao periodica de uma amostra JSON unica no
 * topico carrinho/{id}/telemetria, QoS 0 e sem retencao (secao 4.6).
 *
 * Nada aqui bloqueia o controle (RNF07): o esp-mqtt mantem sua propria tarefa
 * e reconecta sozinho, e a reconexao do Wi-Fi acontece no laco de eventos.
 * Esta tarefa so copia o estado, formata e entrega - se a rede estiver fora,
 * a publicacao falha e a amostra e descartada. Amostra velha nao interessa a
 * ninguem; o cliente quer a mais recente.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"

#include "config.h"
#include "secrets.h"
#include "state.h"
#include "tasks.h"

static const char *TAG = "telemetry";

#define JSON_BUFFER_SIZE 1024

static esp_mqtt_client_handle_t s_mqtt;
static bool s_mqtt_connected;
static uint32_t s_seq;

/* ------------------------------------------------------------------------
 * Wi-Fi
 * --------------------------------------------------------------------- */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    (void)data;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        /* RF11: retomar sozinho quando a rede voltar. */
        ESP_LOGW(TAG, "Wi-Fi caiu, reconectando");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Wi-Fi conectado, IP " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = { 0 };
    strlcpy((char *)wifi_cfg.sta.ssid, WIFI_SSID, sizeof(wifi_cfg.sta.ssid));
    strlcpy((char *)wifi_cfg.sta.password, WIFI_PASSWORD, sizeof(wifi_cfg.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));

    /* Sem economia de energia: o modem dormindo atrasa a publicacao de 200 ms
     * e atrapalha a coexistencia com o Bluetooth. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ------------------------------------------------------------------------
 * MQTT
 * --------------------------------------------------------------------- */

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)data;

    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt_connected = true;
        ESP_LOGI(TAG, "broker conectado, publicando em %s", TELEMETRY_TOPIC);
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_connected = false;
        ESP_LOGW(TAG, "broker desconectado");
        break;
    default:
        break;
    }
}

static void mqtt_start(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    s_mqtt = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt));
}

/* ------------------------------------------------------------------------
 * Serializacao
 * --------------------------------------------------------------------- */

/* Todo acrescimo passa por aqui. snprintf devolve o tamanho que o texto teria
 * sem truncamento, entao pos pode ultrapassar o buffer; a guarda impede que
 * size - pos estoure por baixo, e build_payload confere o total no fim. */
#define APPEND(...)                                                  \
    do {                                                             \
        if (pos < (int)size) {                                       \
            pos += snprintf(buf + pos, size - pos, __VA_ARGS__);     \
        }                                                            \
    } while (0)

/* Distancia invalida vira null, como manda a secao 4.6. */
static int append_distance(char *buf, size_t size, int pos,
                           const char *name, const sonar_state_t *s, bool last)
{
    if (s->valid) {
        APPEND("\"%s\": %.2f%s", name, s->dist_m, last ? "" : ", ");
    } else {
        APPEND("\"%s\": null%s", name, last ? "" : ", ");
    }
    return pos;
}

static int append_age(char *buf, size_t size, int pos, const char *name,
                      const sonar_state_t *s, int64_t now_us, bool last)
{
    if (s->valid) {
        APPEND("\"%s\": %" PRId64 "%s", name, (now_us - s->stamp_us) / 1000,
               last ? "" : ", ");
    } else {
        APPEND("\"%s\": null%s", name, last ? "" : ", ");
    }
    return pos;
}

static int build_payload(char *buf, size_t size, const carrinho_state_t *s)
{
    int64_t now_us = esp_timer_get_time();
    int pos = 0;

    APPEND(
        "{\"schema_versao\": %d, \"seq\": %lu, \"timestamp_ms\": %" PRId64 ", ",
        SCHEMA_VERSAO, (unsigned long)s_seq, now_us / 1000);

    APPEND(
        "\"pose\": {\"x_m\": %.2f, \"y_m\": %.2f, \"yaw_rad\": %.2f}, ",
        s->pose.x_m, s->pose.y_m, s->pose.yaw_rad);

    APPEND(
        "\"velocidade_m_s\": %.2f, ", s->wheel.v_linear_mps);

    APPEND(
        "\"velocidade_lados_m_s\": {\"esquerdo\": %.2f, \"direito\": %.2f}, ",
        s->wheel.v_left_mps, s->wheel.v_right_mps);

    APPEND("\"sensores_m\": {");
    pos = append_distance(buf, size, pos, "frente",   &s->sonar[SONAR_FRENTE],   false);
    pos = append_distance(buf, size, pos, "tras",     &s->sonar[SONAR_TRAS],     false);
    pos = append_distance(buf, size, pos, "esquerda", &s->sonar[SONAR_ESQUERDA], false);
    pos = append_distance(buf, size, pos, "direita",  &s->sonar[SONAR_DIREITA],  true);
    APPEND("}, ");

    APPEND("\"sensores_idade_ms\": {");
    pos = append_age(buf, size, pos, "frente",   &s->sonar[SONAR_FRENTE],   now_us, false);
    pos = append_age(buf, size, pos, "tras",     &s->sonar[SONAR_TRAS],     now_us, false);
    pos = append_age(buf, size, pos, "esquerda", &s->sonar[SONAR_ESQUERDA], now_us, false);
    pos = append_age(buf, size, pos, "direita",  &s->sonar[SONAR_DIREITA],  now_us, true);
    APPEND("}, ");

    APPEND("\"imu\": {");
    if (s->imu.valid) {
        APPEND(
            "\"aceleracao_m_s2\": {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}, "
            "\"giroscopio_rad_s\": {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}, "
            "\"idade_ms\": %" PRId64 ", ",
            s->imu.accel_m_s2[0], s->imu.accel_m_s2[1], s->imu.accel_m_s2[2],
            s->imu.gyro_rad_s[0], s->imu.gyro_rad_s[1], s->imu.gyro_rad_s[2],
            (now_us - s->imu.stamp_us) / 1000);
    } else {
        APPEND(
            "\"aceleracao_m_s2\": null, \"giroscopio_rad_s\": null, "
            "\"idade_ms\": null, ");
    }
    APPEND(
        "\"yaw_integrado_rad\": %.2f, \"yaw_odom_rad\": %.2f, \"valida\": %s}, ",
        s->pose.yaw_rad, s->pose.yaw_odom_rad, s->imu.valid ? "true" : "false");

    APPEND(
        "\"seguranca\": {\"timeout_ativo\": %s, \"frenagem_automatica\": %s, "
        "\"frontal_invalido\": %s}}",
        s->safety.timeout_active ? "true" : "false",
        s->safety.auto_brake     ? "true" : "false",
        s->safety.front_invalid  ? "true" : "false");

    return pos;
}

/* ------------------------------------------------------------------------
 * Tarefa
 * --------------------------------------------------------------------- */

static void telemetry_task(void *arg)
{
    (void)arg;

    static char payload[JSON_BUFFER_SIZE];
    TickType_t next = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(PERIOD_TELEMETRY_MS));

        if (!s_mqtt_connected) {
            continue;
        }

        carrinho_state_t snap;
        state_snapshot(&snap);

        int len = build_payload(payload, sizeof(payload), &snap);
        if (len <= 0 || len >= (int)sizeof(payload)) {
            ESP_LOGE(TAG, "payload truncado (%d bytes)", len);
            continue;
        }

        esp_mqtt_client_publish(s_mqtt, TELEMETRY_TOPIC, payload, len, 0, 0);
        s_seq++;
    }
}

void telemetry_start(void)
{
    wifi_start();
    mqtt_start();

    xTaskCreate(telemetry_task, "telemetry", 5120, NULL, PRIO_TELEMETRY, NULL);
}
