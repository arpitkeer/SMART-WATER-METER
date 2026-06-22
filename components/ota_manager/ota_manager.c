#include "include/ota_manager.h"
#include "../nimble_adv/include/nimble_adv.h"
#include "../watchdog/include/watchdog.h" // Ensure this path matches your project structure

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota_manager";

static esp_ota_handle_t s_ota_handle = 0;
static const esp_partition_t *s_update_partition = NULL;
static uint32_t s_bytes_received = 0;
static uint32_t s_binary_size = 0;
static bool s_ota_running = false;
/* =========================================================
 * OTA STATE VARIABLES
 * ========================================================= */
int ota_control_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) 
{
    // 1. Security Check: Reject if the client hasn't passed wm_auth
    if (!nimble_adv_is_authorized()) {
        ESP_LOGW(TAG, "Blocked OTA command from unauthorized client");
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return 0;
    }

    uint8_t cmd = ctxt->om->om_data[0];

    if (cmd == 0x01) // COMMAND: START OTA
    { 
        if (OS_MBUF_PKTLEN(ctxt->om) < 5) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        memcpy(&s_binary_size, &ctxt->om->om_data[1], 4);
        s_bytes_received = 0;
        
        s_update_partition = esp_ota_get_next_update_partition(NULL);
        if (!s_update_partition) {
            ESP_LOGE(TAG, "No OTA partition found!");
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        printf("The binary size is %" PRIu32 " bytes\n", s_binary_size);
        // Feed watchdog before starting because esp_ota_begin erases the first flash sector (blocking)
        watchdog_feed(); 
        esp_err_t err = esp_ota_begin(s_update_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
        
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
            return BLE_ATT_ERR_UNLIKELY;
        }

        s_ota_running = true;
        printf("[OTA] Ready for data stream.\n");
    } 
    else if (cmd == 0x02) // COMMAND: END OTA
    { 
        if (!s_ota_running) return BLE_ATT_ERR_UNLIKELY;

        printf("\n[OTA]\n[OTA] Transfer Finished. Validating Signature...\n\n");
        s_ota_running = false;

        watchdog_feed();
        if (esp_ota_end(s_ota_handle) == ESP_OK) {
            if (esp_ota_set_boot_partition(s_update_partition) == ESP_OK) {
                printf("[OTA] SUCCESS! Rebooting to new firmware in 1 second...\n");
                nimble_adv_send_text("OTA:SUCCESS");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            } else {
                printf("[OTA] Failed to set boot partition!\n");
                nimble_adv_send_text("OTA:FAIL_BOOT");
            }
        } else {
            printf("[OTA] Firmware Verification Failed! Corrupted binary.\n");
            nimble_adv_send_text("OTA:FAIL_VERIFY");
        }
    }
    return 0;
}

int ota_data_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) 
{
    if (!s_ota_running) {
        return BLE_ATT_ERR_UNLIKELY; // Reject data if START command wasn't sent
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    uint8_t *data_buf = malloc(len);
    if (!data_buf) return BLE_ATT_ERR_INSUFFICIENT_RES;
    
    // Flatten the mbuf chain into our linear buffer
    ble_hs_mbuf_to_flat(ctxt->om, data_buf, len, NULL);

    // Write chunk to flash memory
    esp_err_t err = esp_ota_write(s_ota_handle, data_buf, len);
    free(data_buf);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
        s_ota_running = false; // Abort on write failure
        return BLE_ATT_ERR_UNLIKELY;
    }

    s_bytes_received += len;

    // Feed watchdog so large files don't crash the system
    watchdog_feed(); 

    return 0;
}