#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "../../nimble_adv/include/nimble_adv.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    WM_CMD_SRC_BLE  = 0,
    WM_CMD_SRC_LORA = 1
} wm_cmd_source_t;

void ble_command_handler_set_source(wm_cmd_source_t src);
wm_cmd_source_t ble_command_handler_get_source(void);

esp_err_t lora_manager_init(void);
esp_err_t lora_manager_deinit(void);

/* Periodic meter telemetry */
esp_err_t lora_manager_send_meter_payload(void);

/* Raw frame TX if you need it */
esp_err_t lora_manager_send_frame(const uint8_t *data, size_t len);

/* ASCII response TX back to the LoRa side */
esp_err_t lora_manager_send_text(const char *text);

/* Last complete ASCII line received from LoRa */
bool lora_manager_get_last_rx(char *out, size_t out_len);

typedef struct
{
    bool board_has_lora;
    bool uplink_enabled;
    bool downlink_enabled;
    bool listen_enabled;
    uint32_t listen_ms;
} lora_manager_config_t;

void lora_manager_set_config(const lora_manager_config_t *cfg);
void lora_manager_build_meter_payload(water_meter_payload_t *payload);

/* Source flag used by the common command handler */
void ble_command_handler_set_source(wm_cmd_source_t src);
wm_cmd_source_t ble_command_handler_get_source(void);

#ifdef __cplusplus
}
#endif