#ifndef EEPROM_24CXX_H
#define EEPROM_24CXX_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================
 * EEPROM ADDRESS SIZE
 * ========================================================= */

typedef enum
{
    EEPROM_ADDR_1BYTE = 1,
    EEPROM_ADDR_2BYTE = 2,

} eeprom_addr_bytes_t;

/* =========================================================
 * CONFIG
 * ========================================================= */

typedef struct
{
    i2c_port_num_t port;

    gpio_num_t sda_io;
    gpio_num_t scl_io;

    bool enable_internal_pullups;

    uint32_t scl_speed_hz;

    uint8_t i2c_addr_7bit;

    eeprom_addr_bytes_t addr_bytes;

} eeprom_24cxx_config_t;

/* =========================================================
 * API
 * ========================================================= */

esp_err_t eeprom_24cxx_init(
    const eeprom_24cxx_config_t *cfg);

esp_err_t eeprom_24cxx_read(
    uint16_t mem_addr,
    uint8_t *data,
    size_t len);

esp_err_t eeprom_24cxx_write(
    uint16_t mem_addr,
    const uint8_t *data,
    size_t len);

esp_err_t eeprom_24cxx_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* EEPROM_24CXX_H */