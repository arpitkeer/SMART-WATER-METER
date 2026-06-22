#ifndef BLE_BUFFER_H
#define BLE_BUFFER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

void ble_buffer_init(void);
void ble_buffer_push(const char *msg);
bool ble_buffer_pop(char *out, size_t out_len);
bool ble_buffer_empty(void);

#endif
