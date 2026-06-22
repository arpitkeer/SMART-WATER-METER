#pragma once

#include "host/ble_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handles OTA START (0x01) and OTA END (0x02) commands.
 */
int ota_control_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

/**
 * @brief Handles the high-speed binary stream (Write Without Response).
 */
int ota_data_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

#ifdef __cplusplus
}
#endif