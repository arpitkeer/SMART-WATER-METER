#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <inttypes.h>
#include "../wm_auth/include/wm_auth.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"

#include "nimble/hci_common.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_hci.h"
#include "host/ble_hs_mbuf.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "../deep_sleep_manager/include/deep_sleep_manager.h"
#include "../meter_helper/include/meter_helper.h"
#include "../meter_storage/include/meter_storage.h"
#include "include/nimble_adv.h"

// INCLUDE THE NEW OTA MANAGER
#include "ota_manager.h" 

static const char *TAG = "nimble_adv";

/* =========================================================
 * CONFIG
 * ========================================================= */

#ifndef CONFIG_WM_BLE_NAME
#define CONFIG_WM_BLE_NAME "WM"
#endif

#define DEVICE_NAME CONFIG_WM_BLE_NAME

#define FW_VERSION 1

/* =========================================================
 * STATE
 * ========================================================= */

static bool s_ready = false;
static bool s_connected = false;
static bool s_authorized = false;
static bool s_data_notify_enabled = false;
static bool s_text_notify_enabled = false;
static bool s_adv_running = false;

static uint16_t s_text_handle = 0;
static uint16_t s_conn_handle = 0;
static uint16_t s_data_handle = 0;

static uint8_t s_addr_type;
static uint8_t s_adv_instance = 0;

static ble_cmd_cb_t s_cmd_cb = NULL;

extern int64_t last_activity;
static int64_t s_connect_time_us = 0;
static uint8_t s_auth_fail_count = 0;
/* =========================================================
 * AUTH CHALLENGE
 * ========================================================= */

static uint8_t s_challenge[WM_CHALLENGE_LEN];
static bool s_challenge_ready = false;

/* =========================================================
 * RUNTIME OVERRIDES
 * ========================================================= */

static uint32_t s_total = 0;
static bool s_total_valid = false;

static bool s_case_tamper_override = false;
static bool s_case_tamper_valid = false;

/* Live runtime state lives in main.c */
extern uint32_t rtc_blocks;
extern uint32_t pulse_bucket;

/* =========================================================
 * UUIDS
 * ========================================================= */

static const ble_uuid128_t TEXT_UUID =
    BLE_UUID128_INIT(
        0x03, 0xF0, 0xDE, 0xBC,
        0x9A, 0x78,
        0x56, 0x34,
        0x12, 0xF0,
        0xDE, 0xBC,
        0x9A, 0x78,
        0x56, 0x34);

static const ble_uuid128_t SERVICE_UUID =
    BLE_UUID128_INIT(
        0x11,0x22,0x33,0x44,
        0x55,0x66,
        0x77,0x88,
        0x99,0xAA,
        0xBB,0xCC,0xDD,0xEE,0xFF,0x00);

static const ble_uuid128_t DATA_UUID =
    BLE_UUID128_INIT(
        0x10,0x32,0x54,0x76,
        0x98,0xBA,
        0xDC,0xFE,
        0x12,0x34,
        0x56,0x78,0x9A,0xBC,0xDE,0xF0);

static const ble_uuid128_t CMD_UUID =
    BLE_UUID128_INIT(
        0xAA,0xBB,0xCC,0xDD,
        0xEE,0xFF,
        0x11,0x22,
        0x33,0x44,
        0x55,0x66,0x77,0x88,0x99,0x00);


/* =========================================================
 * FORWARD DECLARATIONS
 * ========================================================= */

static int gap_event(
    struct ble_gap_event *event,
    void *arg);

static void notify_payload(void);
static int send_text_common(
    uint16_t conn_handle,
    const char *msg,
    bool bypass_auth);

static bool advertise(void);
static void reset_connection_state(void);

/* =========================================================
 * HELPERS
 * ========================================================= */

static void generate_new_challenge(void)
{
    esp_fill_random(
        s_challenge,
        WM_CHALLENGE_LEN);

    s_challenge_ready = true;

    printf(
        "\n[AUTH]\n"
        "[AUTH] Challenge: "
        "%02X%02X%02X%02X%02X%02X%02X%02X\n\n",
        s_challenge[0], s_challenge[1],
        s_challenge[2], s_challenge[3],
        s_challenge[4], s_challenge[5],
        s_challenge[6], s_challenge[7]);
}

void put_u24(
    uint8_t *buf,
    uint32_t value)
{
    buf[0] = (value >> 16) & 0xFF;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = value & 0xFF;
}

void put_u40(
    uint8_t *buf,
    uint64_t value)
{
    buf[0] = (value >> 32) & 0xFF;
    buf[1] = (value >> 24) & 0xFF;
    buf[2] = (value >> 16) & 0xFF;
    buf[3] = (value >> 8) & 0xFF;
    buf[4] = value & 0xFF;
}

static uint16_t crc16_ccitt(
    const uint8_t *data,
    size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);

        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

uint8_t build_tamper_fw(
    bool case_tamper,
    bool magnetic_tamper,
    bool removal_tamper)
{
    uint8_t v = 0;

    v |= (case_tamper ? 1 : 0) << 0;
    v |= (magnetic_tamper ? 1 : 0) << 1;
    v |= (removal_tamper ? 1 : 0) << 2;

    v |= (FW_VERSION & 0x1F) << 3;

    return v;
}


void build_payload(
    water_meter_payload_t *p)
{
    meter_storage_record_t nv;
    memset(&nv, 0, sizeof(nv));

    if (meter_storage_get_copy(&nv) != ESP_OK)
    {
        memset(&nv, 0, sizeof(nv));
        strncpy(nv.meter_id, "WM000001", sizeof(nv.meter_id) - 1);
        nv.meter_type = CONFIG_WM_METER_TYPE;
        nv.volume_multiplier = CONFIG_WM_VOLUME_MULTIPLIER;
        nv.install_day = 1;
        nv.install_month = 1;
        nv.install_year = 2026;
        nv.install_counter = 0;
        nv.battery_level = 95;
        nv.rtc_blocks = 0;
        nv.pulse_bucket = 0;
        nv.tamper_case = 0;
        nv.tamper_magnetic = 0;
        nv.tamper_removal = 0;
    }

    memset(p, 0, sizeof(*p));

    /* Always use the live runtime state for telemetry. */
    bool case_tamper = meter_helper_get_case_tamper();
    bool magnetic_tamper = meter_helper_get_magnetic_tamper();
    bool removal_tamper = meter_helper_get_removal_tamper();

    p->tamper_fw =
        build_tamper_fw(
            case_tamper,
            magnetic_tamper,
            removal_tamper);

    p->meter_type = (nv.meter_type != 0) ? nv.meter_type : CONFIG_WM_METER_TYPE;

    put_u40(
        p->timestamp,
        (uint64_t)time(NULL));

    p->volume_multiplier =
        (nv.volume_multiplier != 0)
            ? nv.volume_multiplier
            : CONFIG_WM_VOLUME_MULTIPLIER;

    uint32_t total =
        (rtc_blocks * CONFIG_WM_CYCLIC_RESET_COUNT) + pulse_bucket;

    put_u24(
        p->meter_counter,
        total);

    p->interval_minutes = CONFIG_WM_PERIODIC_INTERVAL_MIN;

    p->battery_level = nv.battery_level;

    p->meter_id_len = (uint8_t)strnlen(nv.meter_id, sizeof(nv.meter_id));

    memcpy(
        p->meter_id,
        nv.meter_id,
        p->meter_id_len);

    p->install_date[0] = nv.install_day;
    p->install_date[1] = nv.install_month;
    p->install_date[2] = (uint8_t)(nv.install_year % 100);

    p->install_counter = nv.install_counter;

    if (s_challenge_ready)
    {
        memcpy(p->challenge, s_challenge, WM_CHALLENGE_LEN);
    }
    else
    {
        memset(p->challenge, 0, WM_CHALLENGE_LEN);
    }

    p->crc = crc16_ccitt(
        (const uint8_t *)p,
        sizeof(*p) - sizeof(p->crc));
}

static void reset_connection_state(void)
{
    s_connected = false;
    s_authorized = false;
    s_data_notify_enabled = false;
    s_text_notify_enabled = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_connect_time_us = 0;
    s_auth_fail_count = 0;
    last_activity = esp_timer_get_time();
}

/* =========================================================
 * GATT ACCESS
 * ========================================================= */

static int data_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (!s_authorized)
    {
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }

    water_meter_payload_t payload;
    build_payload(&payload);

    return os_mbuf_append(
        ctxt->om,
        &payload,
        sizeof(payload));
}

static int cmd_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    char cmd[64];
    memset(cmd, 0, sizeof(cmd));

    uint16_t pkt_len = (uint16_t)OS_MBUF_PKTLEN(ctxt->om);
    if (pkt_len >= sizeof(cmd))
    {
        pkt_len = sizeof(cmd) - 1;
    }

    os_mbuf_copydata(
        ctxt->om,
        0,
        pkt_len,
        cmd);

    cmd[pkt_len] = '\0';

    ESP_LOGI(TAG, "CMD=%s", cmd);

    /* =====================================================
     * AUTH COMMAND
     * Format: AUTH:AABBCCDDEEFF0011
     * ===================================================== */


    if (strncmp(cmd, "AUTH:", 5) == 0)
    {
        const char *hex_response = cmd + 5;

        uint8_t received[WM_RESPONSE_LEN];
        uint8_t expected[WM_RESPONSE_LEN];
        uint8_t per_meter_key[32];

        if (!wm_auth_decode_hex(hex_response, received))
        {
            ESP_LOGW(TAG, "AUTH bad hex");
            s_authorized = false;
            last_activity = esp_timer_get_time();

            printf(
                "\n[AUTH]\n"
                "[AUTH] REJECTED\n\n");

            (void)send_text_common(conn_handle, "AUTH:FAIL", true);

            if (s_auth_fail_count < 255) {
                s_auth_fail_count++;
            }
            if (s_auth_fail_count >= 1) {
                nimble_adv_disconnect();
            }

            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        meter_storage_record_t nv;
        memset(&nv, 0, sizeof(nv));

        if (meter_storage_get_copy(&nv) != ESP_OK)
        {
            strncpy(nv.meter_id, "WM000001", sizeof(nv.meter_id) - 1);
        }

        wm_auth_derive_key(
            nv.meter_id,
            per_meter_key);

        wm_auth_compute_response(
            per_meter_key,
            s_challenge,
            expected);

			if (wm_auth_compare(
			        received,
			        expected,
			        WM_RESPONSE_LEN))
			{
            s_authorized = true;
            s_auth_fail_count = 0;
            ESP_LOGI(TAG, "AUTH SUCCESS");
            last_activity = esp_timer_get_time();

            printf(
                "\n[AUTH]\n"
                "[AUTH] AUTHORIZED\n\n");
				extern int64_t auth_start_us;
				auth_start_us = esp_timer_get_time();
				last_activity = auth_start_us;
			    uint8_t proof[WM_RESPONSE_LEN];
			    char proof_hex[WM_RESPONSE_LEN * 2 + 1];
			    char proof_msg[64];

			    wm_auth_compute_proof(
			        per_meter_key,
			        s_challenge,
			        proof);

			    wm_auth_encode_hex(
			        proof,
			        proof_hex);

			    snprintf(
			        proof_msg,
			        sizeof(proof_msg),
			        "AUTH_PROOF:%s",
			        proof_hex);

			    (void)send_text_common(
			        conn_handle,
			        proof_msg,
			        true);
			}
        else
        {
            s_authorized = false;
            ESP_LOGW(TAG, "AUTH FAILED");
            last_activity = esp_timer_get_time();

            printf(
                "\n[AUTH]\n"
                "[AUTH] REJECTED\n\n");

            (void)send_text_common(conn_handle, "AUTH:FAIL", true);

            if (s_auth_fail_count < 255) {
                s_auth_fail_count++;
            }
            if (s_auth_fail_count >= 1) {
                nimble_adv_disconnect();
            }
        }

        return 0;
    }

    if (s_cmd_cb)
    {
        s_cmd_cb(cmd);
    }

    return 0;
}

/* =========================================================
 * SERVICES
 * ========================================================= */

static int text_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    /* TEXT characteristic is notify-only; reads are not expected.
     * Returning 0 here prevents the CCCD subscription write from
     * being rejected before auth completes. */
    (void)conn_handle;
    (void)attr_handle;
    (void)ctxt;
    (void)arg;
    return 0;
}

/* THE UPDATED GATT TABLE (Includes Custom Water Meter + New OTA Service) */
static const struct ble_gatt_svc_def services[] =
{
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SERVICE_UUID.u,
        .characteristics =
        (struct ble_gatt_chr_def[])
        {
            {
                .uuid = &DATA_UUID.u,
                .access_cb = data_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_data_handle,
            },
            {
                .uuid = &TEXT_UUID.u,
                .access_cb = text_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_text_handle,
            },
            {
                .uuid = &CMD_UUID.u,
                .access_cb = cmd_access,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {0}
        },
    },
    {
        /* === NEW OTA UPDATE SERVICE === */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1234),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                /* OTA Control Characteristic (Start / End) */
                .uuid = BLE_UUID16_DECLARE(0x2345),
                .access_cb = ota_control_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                /* OTA Data Characteristic (High-Speed Binary Streaming) */
                .uuid = BLE_UUID16_DECLARE(0x3456),
                .access_cb = ota_data_cb,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {0}
        },
    },
    {0} // Array Terminator
};


/* =========================================================
 * NOTIFY / TEXT
 * ========================================================= */

static void notify_payload(void)
{
    if (!s_connected ||
        !s_data_notify_enabled ||
        !s_authorized)
    {
        return;
    }

    water_meter_payload_t payload;
    build_payload(&payload);

    struct os_mbuf *om =
        ble_hs_mbuf_from_flat(
            &payload,
            sizeof(payload));

    if (!om)
    {
        return;
    }

    ble_gatts_notify_custom(
        s_conn_handle,
        s_data_handle,
        om);
}

static int send_text_common(
    uint16_t conn_handle,
    const char *msg,
    bool bypass_auth)
{
    if (!msg)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (conn_handle == BLE_HS_CONN_HANDLE_NONE)
    {
        ESP_LOGW(TAG, "TEXT send blocked: invalid conn handle");
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (!bypass_auth)
    {
        if (!s_connected || !s_text_notify_enabled || !s_authorized)
        {
            ESP_LOGW(TAG,
                     "TEXT send blocked: conn=%d text_notify=%d auth=%d bypass=%d",
                     s_connected ? 1 : 0,
                     s_text_notify_enabled ? 1 : 0,
                     s_authorized ? 1 : 0,
                     bypass_auth ? 1 : 0);
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    struct os_mbuf *om =
        ble_hs_mbuf_from_flat(msg, strlen(msg));

    if (!om)
    {
        ESP_LOGW(TAG, "mbuf alloc failed");
        return BLE_ATT_ERR_UNLIKELY;
    }

    int rc = ble_gatts_notify_custom(
        conn_handle,
        s_text_handle,
        om);

    ESP_LOGI(TAG, "notify rc=%d msg=%s", rc, msg);
    return rc;
}

void nimble_adv_send_text(const char *msg)
{
    (void)send_text_common(s_conn_handle, msg, false);
}

void nimble_adv_send_control_text(const char *msg)
{
    (void)send_text_common(s_conn_handle, msg, true);
}

/* =========================================================
 * EXTENDED ADVERTISEMENT
 * ========================================================= */


static bool advertise(void)
{
    if (!s_ready)
    {
        ESP_LOGW(TAG, "BLE not ready yet, advertise deferred");
        return false;
    }

    water_meter_payload_t payload;
    build_payload(&payload);

    struct ble_gap_ext_adv_params params;
    memset(&params, 0, sizeof(params));

    params.connectable = 1;
    params.scannable = 0;
    params.legacy_pdu = 0;
    params.primary_phy = BLE_HCI_LE_PHY_CODED;
    params.secondary_phy = BLE_HCI_LE_PHY_CODED;
    params.sid = 1;

    int rc = ble_gap_ext_adv_configure(
        s_adv_instance,
        &params,
        NULL,
        NULL,
        NULL);

    if (rc != 0)
    {
        ESP_LOGE(TAG, "ext_adv_configure rc=%d", rc);
        return false;
    }

    uint8_t adv_data[128];
    uint16_t idx = 0;

    adv_data[idx++] = 2;
    adv_data[idx++] = BLE_HS_ADV_TYPE_FLAGS;
    adv_data[idx++] = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    uint8_t name_len = (uint8_t)strlen(DEVICE_NAME);
    adv_data[idx++] = name_len + 1;
    adv_data[idx++] = BLE_HS_ADV_TYPE_COMP_NAME;
    memcpy(&adv_data[idx], DEVICE_NAME, name_len);
    idx += name_len;

    uint16_t payload_len = sizeof(payload) + 3;
    adv_data[idx++] = payload_len;
    adv_data[idx++] = BLE_HS_ADV_TYPE_MFG_DATA;
    adv_data[idx++] = 0xFF;
    adv_data[idx++] = 0xFF;
    memcpy(&adv_data[idx], &payload, sizeof(payload));
    idx += sizeof(payload);

    struct os_mbuf *om = ble_hs_mbuf_from_flat(adv_data, idx);
    if (!om)
    {
        ESP_LOGE(TAG, "adv mbuf failed");
        return false;
    }

    rc = ble_gap_ext_adv_set_data(s_adv_instance, om);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ext_adv_set_data rc=%d", rc);
        return false;
    }

    rc = ble_gap_ext_adv_start(s_adv_instance, 0, 0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ext_adv_start rc=%d", rc);
        return false;
    }

    s_adv_running = true;

    ESP_LOGI(TAG, "Extended advertising started (%u bytes)", idx);
    return true;
}

/* =========================================================
 * GAP EVENTS
 * ========================================================= */

static int gap_event(
    struct ble_gap_event *event,
    void *arg)
{
    (void)arg;

    switch (event->type)
    {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0)
            {
                s_connected = true;
                s_authorized = false;
                s_data_notify_enabled = false;
                s_text_notify_enabled = false;
                s_conn_handle = event->connect.conn_handle;
                s_connect_time_us = esp_timer_get_time();
                s_auth_fail_count = 0;
				printf("bleconnected");
				}
				else
				{
                reset_connection_state();
				printf("ble disconnected");

                advertise();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected reason=%d", event->disconnect.reason);
			s_authorized = false;
            reset_connection_state();
            generate_new_challenge();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
        {
            ESP_LOGI(
                TAG,
                "SUBSCRIBE attr=%u cur_notify=%d prev_notify=%d",
                event->subscribe.attr_handle,
                event->subscribe.cur_notify,
                event->subscribe.prev_notify);

            if (event->subscribe.attr_handle == s_text_handle)
            {
                s_text_notify_enabled = event->subscribe.cur_notify;

                ESP_LOGI(
                    TAG,
                    "TEXT notify %s",
                    s_text_notify_enabled ? "ENABLED" : "DISABLED");
            }

            if (event->subscribe.attr_handle == s_data_handle)
            {
                s_data_notify_enabled = event->subscribe.cur_notify;

                ESP_LOGI(
                    TAG,
                    "DATA notify %s",
                    s_data_notify_enabled ? "ENABLED" : "DISABLED");

                if (s_data_notify_enabled)
                {
                    notify_payload();
                }
            }

            return 0;
        }

        default:
            return 0;
    }
}

/* =========================================================
 * HOST TASK
 * ========================================================= */

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
}

/* =========================================================
 * SYNC
 * ========================================================= */

static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_addr_type);

    generate_new_challenge();

    s_ready = true;
    ESP_LOGI(TAG, "BLE READY");
}

/* =========================================================
 * INIT
 * ========================================================= */

esp_err_t nimble_adv_init(
    const nimble_adv_config_t *config)
{
    if (config && config->device_name)
    {
        (void)config;
    }

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(services);
    ble_gatts_add_svcs(services);

    ble_svc_gap_device_name_set(DEVICE_NAME);

    ble_hs_cfg.sync_cb = on_sync;

    ble_att_set_preferred_mtu(524);

    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "NimBLE init complete");

    return ESP_OK;
}

void nimble_adv_deinit(void)
{
    nimble_adv_stop();

    s_connected = false;
    s_authorized = false;
    s_data_notify_enabled = false;
    s_text_notify_enabled = false;
    s_ready = false;
    s_adv_running = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_connect_time_us = 0;
    s_auth_fail_count = 0;

    nimble_port_stop();
    nimble_port_deinit();
}

void nimble_adv_disconnect(void)
{
    if (!s_connected) {
        return;
    }

    ble_gap_terminate(
        s_conn_handle,
        BLE_ERR_REM_USER_CONN_TERM);
}

int64_t nimble_adv_get_connect_time_us(void)
{
    return s_connect_time_us;
}

/* =========================================================
 * PUBLIC API
 * ========================================================= */

bool nimble_adv_start(void)
{
    if (s_adv_running)
    {
        return true;
    }

    return advertise();
}

void nimble_adv_stop(void)
{
    s_authorized = false;
    if (s_adv_running)
    {
        ble_gap_ext_adv_stop(s_adv_instance);
        s_adv_running = false;
    }
}

bool nimble_adv_is_connected(void)
{
    return s_connected;
}

bool nimble_adv_is_authorized(void)
{
    return s_authorized;
}

bool nimble_adv_is_notify_enabled(void)
{
    return s_text_notify_enabled;
}

void nimble_adv_authorize(bool enabled)
{
    s_authorized = enabled;
}

void nimble_adv_set_cmd_cb(
    ble_cmd_cb_t cb)
{
    s_cmd_cb = cb;
}

void nimble_adv_update_data(
    uint32_t total,
    bool tamper)
{
    s_total = total;
    s_total_valid = true;

    s_case_tamper_override = tamper;
    s_case_tamper_valid = true;
}

void nimble_adv_flush(void)
{
    notify_payload();
}

void nimble_adv_get_challenge(
    uint8_t out[WM_CHALLENGE_LEN])
{
    if (!out)
    {
        return;
    }

    memcpy(out, s_challenge, WM_CHALLENGE_LEN);
}