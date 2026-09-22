/* Bluetooth SPP command link. The bluedroid callback only queues bytes; parsing
 * and validation happen in this task. */
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "config.h"
#include "state.h"
#include "tasks.h"

static const char *TAG = "bt_command";

#define CMD_STOP       0x00
#define CMD_LINEAR     0x01
#define CMD_ROTATION   0x02

static QueueHandle_t      s_rx_queue;
static esp_timer_handle_t s_timeout_timer;

static void zero_references(const char *reason)
{
    rover_state_t *st = state_lock();
    st->cmd.v_ref_mps   = 0.0f;
    st->cmd.w_ref_rad_s = 0.0f;
    state_unlock();

    ESP_LOGI(TAG, "references zeroed (%s)", reason);
    control_notify();
}

static void on_command_timeout(void *arg)
{
    (void)arg;

    rover_state_t *st = state_lock();
    st->cmd.v_ref_mps        = 0.0f;
    st->cmd.w_ref_rad_s      = 0.0f;
    st->safety.timeout_active = true;
    state_unlock();

    ESP_LOGW(TAG, "command timeout, stopping");
    control_notify();
}

/* Only valid packets renew the timeout. */
static void renew_timeout(void)
{
    esp_timer_stop(s_timeout_timer);
    esp_timer_start_once(s_timeout_timer, (uint64_t)COMMAND_TIMEOUT_MS * 1000);
}

static void apply_packet(uint8_t id, int8_t value)
{
    rover_state_t *st = state_lock();

    float v_before = st->cmd.v_ref_mps;
    float w_before = st->cmd.w_ref_rad_s;

    switch (id) {
    case CMD_STOP:
        st->cmd.v_ref_mps   = 0.0f;
        st->cmd.w_ref_rad_s = 0.0f;
        break;

    /* Each command updates its own component and keeps the other. */
    case CMD_LINEAR:
        st->cmd.v_ref_mps = (value / 100.0f) * V_MAX_MPS;
        break;

    case CMD_ROTATION:
        st->cmd.w_ref_rad_s = (value / 100.0f) * OMEGA_MAX_RAD_S;
        break;

    default:
        break;
    }

    st->cmd.last_packet_us    = esp_timer_get_time();
    st->safety.timeout_active = false;

    /* The client repeats commands at 10 Hz; log only actual changes. */
    bool changed = (st->cmd.v_ref_mps != v_before) || (st->cmd.w_ref_rad_s != w_before);
    float v_now = st->cmd.v_ref_mps;
    float w_now = st->cmd.w_ref_rad_s;
    state_unlock();

    if (changed) {
        ESP_LOGI(TAG, "packet %02x %+4d  ->  v=%+.3f m/s  w=%+.3f rad/s",
                 id, value, v_now, w_now);
    }

    renew_timeout();

    if (id == CMD_STOP) {
        ESP_LOGI(TAG, "priority stop");
        control_notify();
    }
}

static void feed_byte(uint8_t byte)
{
    static bool    have_id = false;
    static uint8_t pending_id;

    if (!have_id) {
        if (byte != CMD_STOP && byte != CMD_LINEAR && byte != CMD_ROTATION) {
            ESP_LOGW(TAG, "invalid command id 0x%02x dropped", byte);
            return;
        }
        pending_id = byte;
        have_id    = true;
        return;
    }

    have_id = false;

    int8_t value = (int8_t)byte;
    if (pending_id != CMD_STOP && (value < -100 || value > 100)) {
        ESP_LOGW(TAG, "value %d out of range, packet dropped", value);
        return;
    }

    apply_packet(pending_id, value);
}

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        esp_bt_gap_set_device_name(BT_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "ROVER_SPP");
        ESP_LOGI(TAG, "SPP started as \"%s\"", BT_DEVICE_NAME);
        break;

    case ESP_SPP_SRV_OPEN_EVT: {
        rover_state_t *st = state_lock();
        st->cmd.link_connected = true;
        st->cmd.v_ref_mps      = 0.0f;
        st->cmd.w_ref_rad_s    = 0.0f;
        state_unlock();
        renew_timeout();
        ESP_LOGI(TAG, "client connected");
        break;
    }

    case ESP_SPP_CLOSE_EVT: {
        rover_state_t *st = state_lock();
        st->cmd.link_connected = false;
        state_unlock();
        zero_references("connection closed");
        break;
    }

    case ESP_SPP_DATA_IND_EVT:
        for (int i = 0; i < param->data_ind.len; i++) {
            uint8_t b = param->data_ind.data[i];
            xQueueSend(s_rx_queue, &b, 0);
        }
        break;

    default:
        break;
    }
}

static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "paired with %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "pairing failed: %d", param->auth_cmpl.stat);
        }
        break;

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
#endif

    default:
        break;
    }
}

static void bt_command_task(void *arg)
{
    (void)arg;

    uint8_t byte;
    for (;;) {
        if (xQueueReceive(s_rx_queue, &byte, portMAX_DELAY) == pdTRUE) {
            feed_byte(byte);
        }
    }
}

void bt_command_start(void)
{
    s_rx_queue = xQueueCreate(128, sizeof(uint8_t));

    const esp_timer_create_args_t timer_args = {
        .callback = on_command_timeout,
        .name     = "cmd_timeout",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_timeout_timer));

    /* BLE is unused; Wi-Fi coexistence needs the memory. */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_callback));
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_callback));

    esp_spp_cfg_t spp_cfg = {
        .mode              = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size    = 0,
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));

    xTaskCreate(bt_command_task, "bt_command", 4096, NULL, PRIO_BT_COMMAND, NULL);
}
