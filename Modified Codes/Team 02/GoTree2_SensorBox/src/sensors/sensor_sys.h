#pragma once

#include <Arduino.h>

#include "app_state.h"

// Periodic system metrics sampler (heap, RSSI, uptime).

void sensor_sys_update(AppState& state);
