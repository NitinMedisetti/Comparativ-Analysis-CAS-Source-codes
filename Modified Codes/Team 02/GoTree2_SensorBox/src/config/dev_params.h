#pragma once

// Centralized configuration for all tunable parameters used during development
// and lab testing. Changing values here automatically propagates across the
// firmware without hunting through multiple source files.

#define WIFI_SSID          "ESP32-asG2A1H2J3S4S5"
#define WIFI_PASS          "espesp4455"
#define SOFTAP_IP          "192.168.4.1"
#define CAM_IP             "192.168.4.2"

#define GPS_BAUD           9600
#define LIDAR_BAUD         230400
#define US_INTERVAL_MS      60  // HC-SR04 minimum delay between triggers (round-robin single sensor every 60 ms)
#define US_TIMEOUT_US      25000  // cover full 4 m round-trip echo (~23 ms) per datasheet

#define TELEMETRY_HZ       2  // reduce UI bandwidth for mission-critical control
#define SNAPSHOT_HZ        1
#define IMU_INTERVAL_MS    10   // ~100 Hz Euler readout for the BNO055 (50-100 Hz per datasheet)
#define LIDAR_HZ           10  // LD06/LD20 target 9-12 Hz

#define CBOR_BUFFER_SIZE   16384
// Shared I2C bus (OLED + IMU + other peripherals). 400kHz can be unreliable on longer
// wiring / weaker pull-ups; 100kHz is the safer default.
#define OLED_I2C_FREQ      100000
#define I2C_FALLBACK_FREQ   100000

#define BUTTON_DEBOUNCE_MS 60
