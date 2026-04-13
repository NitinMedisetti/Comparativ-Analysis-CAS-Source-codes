#pragma once

#include <Arduino.h>

// Time helpers used across the firmware.
//
// - `now_us()` uses ESP-IDF's microsecond timer (`esp_timer_get_time()`), which is
//   monotonic and suitable for stamps/latency measurements.
// - `now_ms()` is Arduino's `millis()` for coarse scheduling.

uint64_t now_us();
uint64_t since_us(uint64_t start_us);
inline uint32_t now_ms() { return millis(); }
