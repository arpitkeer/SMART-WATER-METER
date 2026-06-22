#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t watchdog_init(uint32_t timeout_sec);

esp_err_t watchdog_feed(void);

esp_err_t watchdog_deinit(void);

void watchdog_print_reset_reason(void);

#ifdef __cplusplus
}
#endif