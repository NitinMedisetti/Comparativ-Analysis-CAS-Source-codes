#pragma once
#include "app_state.h"

// BNO055 IMU wrapper (Euler angles + calibration status).

// Initialise the BNO055 IMU if present on the I2C bus. Returns true when an IMU is
// detected and successfully configured.
bool sensor_imu_begin();

// Refresh the IMU readings and project them into the shared application state.
void sensor_imu_update(AppState& state);

// Re-probe and reinitialise the IMU; clears existing IMU data in state before retrying.
bool sensor_imu_restart(AppState& state);
