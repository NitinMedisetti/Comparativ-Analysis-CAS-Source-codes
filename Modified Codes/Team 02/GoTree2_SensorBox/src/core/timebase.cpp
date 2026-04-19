#include "timebase.h"

#include <esp_timer.h>

uint64_t now_us() {
  return esp_timer_get_time();
}

uint64_t since_us(uint64_t start_us) {
  const uint64_t now = now_us();
  return (now >= start_us) ? (now - start_us) : 0;
}
