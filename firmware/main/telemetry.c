/* The rover is its own network: a Wi-Fi access point with a Mosquitto broker,
 * so a phone or a laptop joins it and subscribes with no router or internet.
 * The local esp-mqtt client publishes one JSON sample per period to that
 * broker, QoS 0, not retained; while offline samples are dropped, not queued. */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mosq_broker.h"
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

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    (void)base;

    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "client joined " MACSTR, MAC2STR(event->mac));
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "client left " MACSTR, MAC2STR(event->mac));
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .ap = {
            .channel        = WIFI_AP_CHANNEL,
            .max_connection = WIFI_AP_MAX_CLIENTS,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };
    strlcpy((char *)wifi_cfg.ap.ssid, WIFI_AP_SSID, sizeof(wifi_cfg.ap.ssid));
    strlcpy((char *)wifi_cfg.ap.password, WIFI_AP_PASSWORD, sizeof(wifi_cfg.ap.password));
    wifi_cfg.ap.ssid_len = strlen(WIFI_AP_SSID);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));

    /* Modem sleep delays publishing and hurts Bluetooth coexistence. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "access point \"%s\" up, broker at 192.168.4.1:%d",
             WIFI_AP_SSID, MQTT_BROKER_PORT);
}

/* mosq_broker_run() serves clients in the calling task and only returns on
 * error. */
static void broker_task(void *arg)
{
    (void)arg;

    struct mosq_broker_config cfg = {
        .host = "0.0.0.0",
        .port = MQTT_BROKER_PORT,
    };
    int err = mosq_broker_run(&cfg);
    ESP_LOGE(TAG, "broker stopped (%d)", err);
    vTaskDelete(NULL);
}

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)data;

    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt_connected = true;
        ESP_LOGI(TAG, "broker connected, publishing to %s", TELEMETRY_TOPIC);
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_connected = false;
        ESP_LOGW(TAG, "broker disconnected");
        break;
    default:
        break;
    }
}

static void mqtt_start(void)
{
    /* The broker task may not be listening yet; retry fast instead of
     * waiting the default 10 s. */
    esp_mqtt_client_config_t cfg = {
        .broker.address = {
            .transport = MQTT_TRANSPORT_OVER_TCP,
            .hostname  = "127.0.0.1",
            .port      = MQTT_BROKER_PORT,
        },
        .network.reconnect_timeout_ms = 1000,
    };

    s_mqtt = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt));
}

/* snprintf returns the untruncated length, so pos may run past the buffer;
 * the guard keeps size - pos from underflowing. */
#define APPEND(...)                                                  \
    do {                                                             \
        if (pos < (int)size) {                                       \
            pos += snprintf(buf + pos, size - pos, __VA_ARGS__);     \
        }                                                            \
    } while (0)

/* Invalid data is serialized as null. */
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

static int build_payload(char *buf, size_t size, const rover_state_t *s)
{
    int64_t now_us = esp_timer_get_time();
    int pos = 0;

    APPEND(
        "{\"schema_version\": %d, \"seq\": %lu, \"timestamp_ms\": %" PRId64 ", ",
        SCHEMA_VERSION, (unsigned long)s_seq, now_us / 1000);

    APPEND(
        "\"pose\": {\"x_m\": %.2f, \"y_m\": %.2f, \"yaw_rad\": %.2f}, ",
        s->pose.x_m, s->pose.y_m, s->pose.yaw_rad);

    APPEND(
        "\"speed_m_s\": %.2f, ", s->wheel.v_linear_mps);

    APPEND(
        "\"wheel_speed_m_s\": {\"left\": %.2f, \"right\": %.2f}, ",
        s->wheel.v_left_mps, s->wheel.v_right_mps);

    APPEND("\"sonar_m\": {");
    pos = append_distance(buf, size, pos, "front", &s->sonar[SONAR_FRONT], false);
    pos = append_distance(buf, size, pos, "rear",  &s->sonar[SONAR_REAR],  false);
    pos = append_distance(buf, size, pos, "left",  &s->sonar[SONAR_LEFT],  false);
    pos = append_distance(buf, size, pos, "right", &s->sonar[SONAR_RIGHT], true);
    APPEND("}, ");

    APPEND("\"sonar_age_ms\": {");
    pos = append_age(buf, size, pos, "front", &s->sonar[SONAR_FRONT], now_us, false);
    pos = append_age(buf, size, pos, "rear",  &s->sonar[SONAR_REAR],  now_us, false);
    pos = append_age(buf, size, pos, "left",  &s->sonar[SONAR_LEFT],  now_us, false);
    pos = append_age(buf, size, pos, "right", &s->sonar[SONAR_RIGHT], now_us, true);
    APPEND("}, ");

    APPEND("\"imu\": {");
    if (s->imu.valid) {
        APPEND(
            "\"accel_m_s2\": {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}, "
            "\"gyro_rad_s\": {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}, "
            "\"age_ms\": %" PRId64 ", ",
            s->imu.accel_m_s2[0], s->imu.accel_m_s2[1], s->imu.accel_m_s2[2],
            s->imu.gyro_rad_s[0], s->imu.gyro_rad_s[1], s->imu.gyro_rad_s[2],
            (now_us - s->imu.stamp_us) / 1000);
    } else {
        APPEND(
            "\"accel_m_s2\": null, \"gyro_rad_s\": null, "
            "\"age_ms\": null, ");
    }
    APPEND(
        "\"yaw_fused_rad\": %.2f, \"yaw_odom_rad\": %.2f, \"valid\": %s}, ",
        s->pose.yaw_rad, s->pose.yaw_odom_rad, s->imu.valid ? "true" : "false");

    APPEND(
        "\"safety\": {\"timeout_active\": %s}}",
        s->safety.timeout_active ? "true" : "false");

    return pos;
}

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

        rover_state_t snap;
        state_snapshot(&snap);

        int len = build_payload(payload, sizeof(payload), &snap);
        if (len <= 0 || len >= (int)sizeof(payload)) {
            ESP_LOGE(TAG, "payload truncated (%d bytes)", len);
            continue;
        }

        esp_mqtt_client_publish(s_mqtt, TELEMETRY_TOPIC, payload, len, 0, 0);
        s_seq++;
    }
}

void telemetry_start(void)
{
    wifi_start();
    xTaskCreate(broker_task, "mqtt_broker", 6144, NULL, PRIO_TELEMETRY, NULL);
    mqtt_start();

    xTaskCreate(telemetry_task, "telemetry", 5120, NULL, PRIO_TELEMETRY, NULL);
}
