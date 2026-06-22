#include "include/lora_manager.h"
#include "../nimble_adv/include/nimble_adv.h"
#include "../meter_helper/include/meter_helper.h"
#include "../meter_storage/include/meter_storage.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <time.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "adc_monitor.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"

static const char *TAG = "lora_manager";

/* =========================================================
 * GPIO / UART CONFIG
 * ========================================================= */

#define LORA_WAKE_GPIO          GPIO_NUM_3
#define LORA_TX_GPIO            GPIO_NUM_4
#define LORA_RX_GPIO            GPIO_NUM_1
#define LORA_UART               UART_NUM_1

#define LORA_BAUD_RATE          9600

#define LORA_WAKE_DELAY1_MS     150
#define LORA_WAKE_DELAY2_MS     150
#define LORA_WAKE_DELAY3_MS     200
#define LORA_POST_SEND_GAP_MS   50

/* Buffers */
#define LORA_UART_RX_BUF_SIZE   512
#define LORA_UART_TX_BUF_SIZE   512
#define LORA_RX_LINE_MAX        128
#define LORA_TX_LINE_MAX        128
#define LORA_FRAME_MAX          256

#define LORA_FRAME_TYPE_UPLINK  0x01

static SemaphoreHandle_t s_lock = NULL;
static bool s_inited = false;

/* RX line storage */
static char   s_last_rx_line[LORA_RX_LINE_MAX];
static size_t s_last_rx_len = 0;
static bool   s_last_rx_ready = false;

/* Source flag for the shared command handler */
extern void ble_command_handler_set_source(wm_cmd_source_t src);
extern wm_cmd_source_t ble_command_handler_get_source(void);

/* Shared handler lives in meter_helper.c */
extern void ble_command_handler(const char *cmd);

extern uint32_t rtc_blocks;
extern uint32_t pulse_bucket;
extern bool cfg_command;

/* =========================================================
 * CUSTOM LORA PAYLOAD DEFINITION
 * ========================================================= */

#pragma pack(push, 1)
typedef struct {
	uint8_t tamper_fw;
	uint8_t meter_type;
	uint8_t timestamp[5];
	uint8_t volume_multiplier;
	uint8_t meter_counter[3];
	uint8_t battery_level;
	uint8_t meter_id_len;
	char meter_id[16];
	uint8_t install_date[3];
} lora_payload_t;
#pragma pack(pop)

/* =========================================================
 * SMALL HELPERS
 * ========================================================= */

 static lora_manager_config_t s_cfg = {
     .board_has_lora = true,
     .uplink_enabled = true,
     .downlink_enabled = true,
     .listen_enabled = true,
     .listen_ms = 10000,
 };

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

static bool is_hex_string(const char *s, size_t len)
{
    if (!s || len < 2 || (len % 2) != 0) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        if (!isxdigit((unsigned char)s[i])) {
            return false;
        }
    }

    return true;
}

static size_t decode_hex_string(const char *hex, size_t hex_len, uint8_t *out, size_t out_max)
{
    if (!hex || !out || out_max == 0 || (hex_len % 2) != 0) {
        return 0;
    }

    size_t out_len = hex_len / 2;
    if (out_len > out_max) {
        return 0;
    }

    for (size_t i = 0; i < out_len; i++) {
        int hi = hex_nibble(hex[i * 2]);
        int lo = hex_nibble(hex[i * 2 + 1]);

        if (hi < 0 || lo < 0) {
            return 0;
        }

        out[i] = (uint8_t)((hi << 4) | lo);
    }

    return out_len;
}

static bool buffer_is_printable_ascii(const uint8_t *buf, size_t len)
{
    if (!buf || len == 0) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        uint8_t c = buf[i];
        if (c == 0) {
            return false;
        }
        if (c < 32 || c > 126) {
            return false;
        }
    }

    return true;
}

static void lora_capture_last_rx_line(const char *line)
{
    if (!line) {
        return;
    }

    memset(s_last_rx_line, 0, sizeof(s_last_rx_line));
    strlcpy(s_last_rx_line, line, sizeof(s_last_rx_line));
    s_last_rx_len = strnlen(s_last_rx_line, sizeof(s_last_rx_line));
    s_last_rx_ready = (s_last_rx_len > 0);
}

/* 03 FF FF binary config packet — parse fields, fire text commands */
static void lora_handle_binary_cfg_packet(const uint8_t *pkt, size_t len)
{
    if (len < 3 || pkt[0] != 0x03 || pkt[1] != 0xFF || pkt[2] != 0xFF) {
        ESP_LOGW(TAG, "CFG: bad header");
        return;
    }
	cfg_command = true;

    const uint8_t *p   = pkt + 3;
    size_t         rem = len - 3;

    if (rem < 1) { ESP_LOGW(TAG, "CFG: too short"); return; }
    uint8_t id_len = *p++; rem--;
    if (id_len == 0 || id_len > 16 || rem < id_len) { ESP_LOGW(TAG, "CFG: bad id_len"); return; }
    char meter_id[17];
    memcpy(meter_id, p, id_len);
    meter_id[id_len] = '\0';
    p += id_len; rem -= id_len;

    if (rem < 23) { ESP_LOGW(TAG, "CFG: tail too short"); return; }

    uint8_t  meter_type        = *p++; rem--;
    uint8_t  volume_multiplier = *p++; rem--;
    uint32_t install_counter   = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
    p += 3; rem -= 3;
    uint8_t  day   = *p++; rem--;
    uint8_t  month = *p++; rem--;
    uint16_t year  = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
    p += 2; rem -= 2;
    char lat_str[8]; memcpy(lat_str, p, 7); lat_str[7] = '\0'; p += 7; rem -= 7;
    char lon_str[8]; memcpy(lon_str, p, 7); lon_str[7] = '\0';

    ESP_LOGI(TAG, "CFG: id=%s type=%u mult=%u counter=%lu date=%02u/%02u/%04u lat=%s lon=%s",
             meter_id, meter_type, volume_multiplier, (unsigned long)install_counter,
             day, month, year, lat_str, lon_str);

    char cmd[64];

    ble_command_handler_set_source(WM_CMD_SRC_LORA);

    snprintf(cmd, sizeof(cmd), "SET_METER_ID %s", meter_id);
    ble_command_handler(cmd);

    snprintf(cmd, sizeof(cmd), "SET_METER_TYPE %u", meter_type);
    ble_command_handler(cmd);

    snprintf(cmd, sizeof(cmd), "SET_MULTIPLIER %u", volume_multiplier);
    ble_command_handler(cmd);

    snprintf(cmd, sizeof(cmd), "SET_INSTALL_COUNTER %lu", (unsigned long)install_counter);
    ble_command_handler(cmd);

    snprintf(cmd, sizeof(cmd), "SET_INSTALL_DATE %u %u %u", day, month, year);
    ble_command_handler(cmd);

    snprintf(cmd, sizeof(cmd), "SET_LOCATION %s %s", lat_str, lon_str);
    ble_command_handler(cmd);

    snprintf(cmd, sizeof(cmd), "SAVE");
    ble_command_handler(cmd);
	cfg_command = false;

    ble_command_handler_set_source(WM_CMD_SRC_BLE);
}

static void lora_dispatch_command_line(const char *line)
{
    if (!line || !line[0]) {
        return;
    }

    size_t line_len = strnlen(line, LORA_RX_LINE_MAX - 1);

    ESP_LOGI(TAG, "RAW LINE: [%s]", line);
    ESP_LOG_BUFFER_HEX(TAG, (const uint8_t *)line, line_len);

    char cmd[LORA_RX_LINE_MAX];
    memset(cmd, 0, sizeof(cmd));

    /* Binary config packet: 03 FF FF ... — handle before anything else */
    if (line_len >= 3 &&
        (unsigned char)line[0] == 0x03 &&
        (unsigned char)line[1] == 0xFF &&
        (unsigned char)line[2] == 0xFF)
    {
        lora_handle_binary_cfg_packet((const uint8_t *)line, line_len);
        return;
    }

    /* Hex-encoded binary config packet: "03FFFF..." */
    if (is_hex_string(line, line_len) && line_len >= 6) {
        uint8_t peek[3];
        int h0 = hex_nibble(line[0]), l0 = hex_nibble(line[1]);
        int h1 = hex_nibble(line[2]), l1 = hex_nibble(line[3]);
        int h2 = hex_nibble(line[4]), l2 = hex_nibble(line[5]);
        if (h0 >= 0 && l0 >= 0 && h1 >= 0 && l1 >= 0 && h2 >= 0 && l2 >= 0) {
            peek[0] = (uint8_t)((h0 << 4) | l0);
            peek[1] = (uint8_t)((h1 << 4) | l1);
            peek[2] = (uint8_t)((h2 << 4) | l2);
            if (peek[0] == 0x03 && peek[1] == 0xFF && peek[2] == 0xFF) {
                uint8_t decoded[LORA_RX_LINE_MAX];
                size_t dec_len = decode_hex_string(line, line_len, decoded, sizeof(decoded));
                if (dec_len >= 3) {
                    lora_handle_binary_cfg_packet(decoded, dec_len);
                }
                return;
            }
        }
    }

    /* Case 1: raw bytes/string with leading 0x03 or 0x02 */
    if ((unsigned char)line[0] == 0x03 || (unsigned char)line[0] == 0x02) {
        strlcpy(cmd, &line[1], sizeof(cmd));
    }
    /* Case 2: ASCII hex string like 03434C525F54414D504552 */
    else if (is_hex_string(line, line_len)) {
        uint8_t decoded[LORA_RX_LINE_MAX];
        size_t dec_len = decode_hex_string(line, line_len, decoded, sizeof(decoded));

        if (dec_len == 0) {
            ESP_LOGW(TAG, "hex decode failed");
            return;
        }

        ESP_LOG_BUFFER_HEX(TAG, decoded, dec_len);

        /* strip leading 0x03 or 0x02 if present */
        if (dec_len > 0 && (decoded[0] == 0x03 || decoded[0] == 0x02)) {
            memmove(decoded, decoded + 1, dec_len - 1);
            dec_len--;
        }

        if (dec_len == 0) {
            ESP_LOGW(TAG, "decoded command is empty");
            return;
        }

        if (!buffer_is_printable_ascii(decoded, dec_len)) {
            ESP_LOGW(TAG, "decoded bytes are not printable ASCII");
            return;
        }

        size_t copy_len = (dec_len < (sizeof(cmd) - 1)) ? dec_len : (sizeof(cmd) - 1);
        memcpy(cmd, decoded, copy_len);
        cmd[copy_len] = '\0';
    }
    /* Case 3: plain text command */
    else {
        strlcpy(cmd, line, sizeof(cmd));
    }

    if (!cmd[0]) {
        ESP_LOGW(TAG, "empty command after parsing");
        return;
    }

    ESP_LOGI(TAG, "LoRa CMD => [%s]", cmd);

    lora_capture_last_rx_line(cmd);

    ble_command_handler_set_source(WM_CMD_SRC_LORA);
    ble_command_handler(cmd);
    ble_command_handler_set_source(WM_CMD_SRC_BLE);
}

static esp_err_t lora_gpio_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LORA_WAKE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_set_level(LORA_WAKE_GPIO, 0);
    return ESP_OK;
}

static esp_err_t lora_uart_init_internal(void)
{
    uart_config_t cfg = {
        .baud_rate = LORA_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(
        LORA_UART,
        LORA_UART_RX_BUF_SIZE,
        LORA_UART_TX_BUF_SIZE,
        0,
        NULL,
        0);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(LORA_UART, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(
        LORA_UART,
        LORA_TX_GPIO,
        LORA_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART init OK (RX=%d TX=%d)", LORA_UART_RX_BUF_SIZE, LORA_UART_TX_BUF_SIZE);
    return ESP_OK;
}

static void lora_wake_sequence(void)
{
    gpio_set_level(LORA_WAKE_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(LORA_WAKE_DELAY1_MS));

    gpio_set_level(LORA_WAKE_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(LORA_WAKE_DELAY2_MS));

    gpio_set_level(LORA_WAKE_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(LORA_WAKE_DELAY3_MS));

    gpio_set_level(LORA_WAKE_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static esp_err_t lora_uart_send_raw(const uint8_t *data, size_t len)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    int written = uart_write_bytes(LORA_UART, (const char *)data, len);
    if (written < 0) {
        ESP_LOGE(TAG, "uart_write_bytes failed");
        return ESP_FAIL;
    }

    if ((size_t)written != len) {
        ESP_LOGW(TAG, "partial write: %d/%u", written, (unsigned)len);
    }

    esp_err_t ret = uart_wait_tx_done(LORA_UART, pdMS_TO_TICKS(5000));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "uart_wait_tx_done failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

static esp_err_t lora_send_ascii_line_internal(const char *text)
{
    if (!text) {
        return ESP_ERR_INVALID_ARG;
    }

    char line[LORA_TX_LINE_MAX + 3];
    size_t n = 0;

    /* Response frame type */
    line[n++] = 0x01;

    size_t text_len = strnlen(text, LORA_TX_LINE_MAX);

    memcpy(&line[n], text, text_len);
    n += text_len;

    line[n++] = '\r';
    line[n++] = '\n';

    return lora_uart_send_raw((const uint8_t *)line, n);
}

static void lora_flush_pending_line(char *rx_line, size_t *rx_len)
{
    if (!rx_line || !rx_len || *rx_len == 0) {
        return;
    }

    rx_line[*rx_len] = '\0';
    ESP_LOGI(TAG, "LoRa RX CMD (flush): %s", rx_line);
    lora_dispatch_command_line(rx_line);

    *rx_len = 0;
    memset(rx_line, 0, LORA_RX_LINE_MAX);
}

static esp_err_t lora_listen_for_commands(uint32_t listen_ms)
{
    uint64_t deadline_us = esp_timer_get_time() + ((uint64_t)listen_ms * 1000ULL);

    char rx_line[LORA_RX_LINE_MAX];
    size_t rx_len = 0;
    memset(rx_line, 0, sizeof(rx_line));

    size_t buffered = 0;
    uart_get_buffered_data_len(LORA_UART, &buffered);

    ESP_LOGI(TAG, "LoRa listen mode for %u ms", (unsigned)listen_ms);
    ESP_LOGI(TAG, "UART buffered before listen = %u", (unsigned)buffered);

    while ((int64_t)(deadline_us - esp_timer_get_time()) > 0) {
        uint8_t chunk[64];
        int rx = uart_read_bytes(
            LORA_UART,
            chunk,
            sizeof(chunk),
            pdMS_TO_TICKS(100));

        if (rx <= 0) {
            continue;
        }

        /* Any activity extends listen mode another 10 sec from now */
        deadline_us = esp_timer_get_time() + ((uint64_t)listen_ms * 1000ULL);

        ESP_LOGI(TAG, "UART RX LEN=%d", rx);
        ESP_LOG_BUFFER_HEX(TAG, chunk, rx);

        if (rx > 1 && (chunk[0] == 0x03 || chunk[0] == 0x02))
        {
            if ((size_t)rx >= 3 && chunk[0] == 0x03 && chunk[1] == 0xFF && chunk[2] == 0xFF) {
                lora_handle_binary_cfg_packet(chunk, (size_t)rx);
                continue;
            }
            char cmd[64];
            size_t cmd_len = rx - 1;
            memcpy(cmd, &chunk[1], cmd_len);
            cmd[cmd_len] = '\0';
            ESP_LOGI(TAG, "LORA CMD = [%s]", cmd);
            lora_dispatch_command_line(cmd);
            continue;
        }
    }

    /* Flush anything that arrived without newline */
    if (rx_len > 0) {
        lora_flush_pending_line(rx_line, &rx_len);
    }

    ESP_LOGI(TAG, "LoRa listen mode ended");
    return ESP_OK;
}

static void lora_pins_safe_for_sleep(void)
{
    gpio_reset_pin(LORA_TX_GPIO);
    gpio_set_direction(LORA_TX_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_TX_GPIO, 0);

    gpio_reset_pin(LORA_RX_GPIO);
    gpio_set_direction(LORA_RX_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_RX_GPIO, GPIO_FLOATING);

    gpio_reset_pin(LORA_WAKE_GPIO);
    gpio_set_direction(LORA_WAKE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_WAKE_GPIO, 0);

    ESP_LOGI(TAG, "LoRa pins safe for deep sleep");
}

static esp_err_t lora_sleep_sequence(void)
{
    uint8_t sleep_cmd[] = { 0x05, 0x01, 0x02 };

    esp_err_t ret = lora_uart_send_raw(sleep_cmd, sizeof(sleep_cmd));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sleep cmd send failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(LORA_POST_SEND_GAP_MS));
    return ESP_OK;
}

static void build_lora_payload(lora_payload_t *p)
{
    memset(p, 0, sizeof(*p));
    
    meter_storage_record_t nv;
    if (meter_storage_get_copy(&nv) != ESP_OK) {
        nv.meter_type = 0xAC;
        nv.battery_level = 0;
    }
	bool case_tamper = meter_helper_get_case_tamper();
	bool magnetic_tamper = meter_helper_get_magnetic_tamper();
	bool removal_tamper = meter_helper_get_removal_tamper();
	p->tamper_fw =
	        build_tamper_fw(
	            case_tamper,
	            magnetic_tamper,
	            removal_tamper);
    
    p->meter_type = nv.meter_type;
	put_u40( p->timestamp, (uint64_t)time(NULL));
	p->volume_multiplier =(nv.volume_multiplier != 0) ? nv.volume_multiplier: CONFIG_WM_VOLUME_MULTIPLIER;
	uint32_t total = (rtc_blocks * CONFIG_WM_CYCLIC_RESET_COUNT) + pulse_bucket;
	put_u24( p->meter_counter, total);
    p->battery_level = nv.battery_level;
	p->meter_id_len = (uint8_t)strnlen(nv.meter_id, sizeof(nv.meter_id));
	memcpy(p->meter_id,nv.meter_id,p->meter_id_len);

	p->install_date[0] = nv.install_day;
	p->install_date[1] = nv.install_month;
	p->install_date[2] = (uint8_t)(nv.install_year % 100);


}

/* =========================================================
 * PUBLIC API
 * ========================================================= */

esp_err_t lora_manager_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
    }

    if (!s_lock) {
        ESP_LOGE(TAG, "mutex create failed");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = lora_gpio_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = lora_uart_init_internal();
    if (ret != ESP_OK) {
        return ret;
    }

    s_inited = true;
    ESP_LOGI(TAG, "LoRa manager initialized");
    return ESP_OK;
}

esp_err_t lora_manager_deinit(void)
{
    if (!s_inited) {
        return ESP_OK;
    }

    uart_wait_tx_done(LORA_UART, pdMS_TO_TICKS(1000));
    uart_driver_delete(LORA_UART);
    lora_pins_safe_for_sleep();

    s_inited = false;
    ESP_LOGI(TAG, "LoRa deinit complete");
    return ESP_OK;
}

esp_err_t lora_manager_send_text(const char *text)
{
    if (!s_inited || !text) {
        return ESP_ERR_INVALID_STATE;
    }

    return lora_send_ascii_line_internal(text);
}

bool lora_manager_get_last_rx(char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return false;
    }

    if (!s_last_rx_ready) {
        out[0] = '\0';
        return false;
    }

    strlcpy(out, s_last_rx_line, out_len);
    s_last_rx_ready = false;
    s_last_rx_len = 0;
    s_last_rx_line[0] = '\0';
    return true;
}

static uint32_t lora_listen_window_ms(void)
{
    return (s_cfg.listen_enabled ? s_cfg.listen_ms : 0);
}

esp_err_t lora_manager_send_frame(const uint8_t *data, size_t len)
{
    ESP_LOGI(TAG, "========== LORA SEND START ==========");

    if (!s_inited) {
        ESP_LOGE(TAG, "NOT INITIALIZED");
        return ESP_ERR_INVALID_STATE;
    }

    if (!data || len == 0) {
        ESP_LOGE(TAG, "INVALID ARG");
        return ESP_ERR_INVALID_ARG;
    }

    if (len > LORA_FRAME_MAX) {
        ESP_LOGE(TAG, "frame too large");
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(3000)) != pdTRUE) {
        ESP_LOGE(TAG, "Mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    int64_t t0 = esp_timer_get_time();

    ESP_LOGI(TAG, "STEP 1: Wake sequence");
    lora_wake_sequence();

    ESP_LOGI(TAG, "STEP 2: Sending frame len=%u", (unsigned)len);
    esp_err_t ret = lora_uart_send_raw(data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uplink TX failed: %s", esp_err_to_name(ret));
        xSemaphoreGive(s_lock);
        return ret;
    }

    ESP_LOGI(TAG, "STEP 3: Listen window open");
    uint32_t listen_ms = lora_listen_window_ms();
    if (listen_ms > 0)
    {
        ret = lora_listen_for_commands(listen_ms);
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "listen window returned: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "STEP 4: Sending sleep command");
    ret = lora_sleep_sequence();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sleep sequence failed: %s", esp_err_to_name(ret));
    }

    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG, "========== LORA SEND END (%lld ms) ==========",
             (long long)((esp_timer_get_time() - t0) / 1000));

    return ESP_OK;
}

esp_err_t lora_manager_send_meter_payload(void)
{
    if (!s_cfg.board_has_lora || !s_cfg.uplink_enabled)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    lora_payload_t payload;
    build_lora_payload(&payload);

    uint8_t frame[1 + sizeof(payload)];
    frame[0] = LORA_FRAME_TYPE_UPLINK;
    memcpy(&frame[1], &payload, sizeof(payload));

    return lora_manager_send_frame(frame, sizeof(frame));
}

void lora_manager_set_config(const lora_manager_config_t *cfg)
{
    if (cfg)
    {
        s_cfg = *cfg;
    }
}