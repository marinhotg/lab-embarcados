/* Wi-Fi station plus one JSON sample per period, QoS 0, not retained. esp-mqtt
 * runs its own task and reconnects by itself; while offline samples are
 * dropped, not queued. */
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
static esp_timer_handle_t s_wifi_retry;
static bool s_mqtt_connected;
static uint32_t s_seq;

static void wifi_retry(void *arg)
{
    (void)arg;
    esp_wifi_connect();
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    (void)data;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        /* Back off: an immediate retry fails at once and re-fires this event,
         * flooding the log. */
        static int64_t last_warning_us;
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_warning_us > 10000000) {
            ESP_LOGW(TAG, "Wi-Fi down, retrying every %d s",
                     WIFI_RETRY_MS / 1000);
            last_warning_us = now_us;
        }
        esp_timer_stop(s_wifi_retry);
        esp_timer_start_once(s_wifi_retry, (uint64_t)WIFI_RETRY_MS * 1000);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Wi-Fi connected, IP " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void wifi_start(void)
{
    const esp_timer_create_args_t args = { .callback = wifi_retry, .name = "wifi_retry" };
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_wifi_retry));

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

    /* Modem sleep delays publishing and hurts Bluetooth coexistence. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
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
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
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
        "\"safety\": {\"timeout_active\": %s, \"auto_brake\": %s, "
        "\"front_invalid\": %s}}",
        s->safety.timeout_active ? "true" : "false",
        s->safety.auto_brake     ? "true" : "false",
        s->safety.front_invalid  ? "true" : "false");

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
    mqtt_start();

    xTaskCreate(telemetry_task, "telemetry", 5120, NULL, PRIO_TELEMETRY, NULL);
}
