#include "ble_buffer.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#define BLE_BUFFER_SIZE 64
#define BLE_MSG_SIZE    128

typedef struct
{
    char msg[BLE_MSG_SIZE];
} ble_record_t;

static ble_record_t s_buf[BLE_BUFFER_SIZE];
static uint16_t s_wr = 0;
static uint16_t s_rd = 0;
static uint16_t s_count = 0;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

void ble_buffer_init(void)
{
    portENTER_CRITICAL(&s_lock);
    s_wr = 0;
    s_rd = 0;
    s_count = 0;
    memset(s_buf, 0, sizeof(s_buf));
    portEXIT_CRITICAL(&s_lock);
}

void ble_buffer_push(const char *msg)
{
    if (!msg)
    {
        return;
    }

    portENTER_CRITICAL(&s_lock);

    memset(s_buf[s_wr].msg, 0, BLE_MSG_SIZE);
    strlcpy(s_buf[s_wr].msg, msg, BLE_MSG_SIZE);

    s_wr = (uint16_t)((s_wr + 1) % BLE_BUFFER_SIZE);
    if (s_count < BLE_BUFFER_SIZE)
    {
        s_count++;
    }
    else
    {
        s_rd = (uint16_t)((s_rd + 1) % BLE_BUFFER_SIZE);
    }

    portEXIT_CRITICAL(&s_lock);
}

bool ble_buffer_pop(char *out, size_t out_len)
{
    if (!out || out_len == 0)
    {
        return false;
    }

    portENTER_CRITICAL(&s_lock);

    if (s_count == 0)
    {
        portEXIT_CRITICAL(&s_lock);
        return false;
    }

    strlcpy(out, s_buf[s_rd].msg, out_len);
    s_buf[s_rd].msg[0] = '\0';
    s_rd = (uint16_t)((s_rd + 1) % BLE_BUFFER_SIZE);
    s_count--;

    portEXIT_CRITICAL(&s_lock);
    return true;
}

bool ble_buffer_empty(void)
{
    bool empty;
    portENTER_CRITICAL(&s_lock);
    empty = (s_count == 0);
    portEXIT_CRITICAL(&s_lock);
    return empty;
}
