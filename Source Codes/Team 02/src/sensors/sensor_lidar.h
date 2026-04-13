#pragma once
#include "app_state.h"
#include "config/dev_params.h"

// LiDAR UART reader + 360-degree binning for UI/obstacle logic.

// LiDAR UART baud rate (asserted at boot)
#ifndef LIDAR_UART_BAUD
#define LIDAR_UART_BAUD LIDAR_BAUD
#endif

constexpr uint8_t kLidarRawPoints = 12;

struct LidarRawPoint {
  float deg = 0.0f;
  uint16_t mm = 0;
  uint8_t intensity = 0;
};

struct LidarRawFrame {
  uint32_t ts_ms = 0;
  uint8_t rpm = 0;
  uint8_t count = 0;
  LidarRawPoint points[kLidarRawPoints];
};

// Configure the LiDAR interface and prepare the UART used by the ranging sensor.
// Must be invoked during setup before attempting to poll data frames.
void sensor_lidar_begin(int rx_pin, int tx_pin);

// Poll the LiDAR for fresh data and merge the results into `AppState`. Should be
// invoked regularly from the main loop to keep the 360-degree scan up to date.
void sensor_lidar_poll(AppState& state);

// Returns the most recent raw LiDAR packet (body-frame angles).
LidarRawFrame sensor_lidar_last_raw();

void sensor_lidar_debug_request_once();
void sensor_lidar_debug_set_stream(bool enable);
bool sensor_lidar_debug_streaming();
void sensor_lidar_debug_set_raw_stream(bool enable);
bool sensor_lidar_debug_raw_streaming();
