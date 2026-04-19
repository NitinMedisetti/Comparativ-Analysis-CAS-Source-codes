#pragma once
#include "app_state.h"

// Ultrasonic sensor driver (HC-SR04 style) with optional per-sensor enable mask.

struct SensorUSPins {
  int trig_front;
  int echo_front;
  int trig_back;
  int echo_back;
  int trig_left;
  int echo_left;
  int trig_right;
  int echo_right;
  int trig_diag45;
  int echo_diag45;
};

// Store the ultrasonic pin map so readings can be captured during the loop.
void sensor_us_begin(const SensorUSPins& pins);

// Configure which ultrasonic sensors are active (measured) on each update.
// Bit 0: Front, Bit 1: Back, Bit 2: Left, Bit 3: Right, Bit 4: Diag45.
void sensor_us_set_active(bool f, bool b, bool l, bool r, bool d);

// Sample each ultrasonic ranger sequentially and update the shared application state.
void sensor_us_update(AppState& state);
