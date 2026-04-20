#pragma once
#include <TinyGPSPlus.h>
#include "app_state.h"
#include "config/dev_params.h"

// GPS UART setup + TinyGPS++ parsing.

// GPS UART baud rate (asserted at boot)
#ifndef GPS_UART_BAUD
#define GPS_UART_BAUD GPS_BAUD
#endif

// Configure the UART link to the GPS module and push minimal UBX settings.
void sensor_gps_begin(int rx_pin, int tx_pin);

// Drain bytes from the GPS serial link and feed them into the TinyGPS++ parser.
void sensor_gps_poll(TinyGPSPlus& parser);

// Update the shared application state with the latest GPS solution.
// The parser reference is non-const because TinyGPS++ accessors are not const-correct.
void sensor_gps_update_state(TinyGPSPlus& parser, AppState& state);
