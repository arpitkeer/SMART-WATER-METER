#pragma once

#include <stdint.h>
#include <stdbool.h>

void deep_sleep_enter_for_us(uint64_t sleep_us, bool use_gpio_wake);
void deep_sleep_enter(int gpio1, int gpio2, int stuck_gpio);
void deep_sleep_handle_wakeup(int gpio1, int gpio2);