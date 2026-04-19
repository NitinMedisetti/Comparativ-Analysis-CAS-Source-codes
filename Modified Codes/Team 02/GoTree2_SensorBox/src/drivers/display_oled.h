#pragma once
#include <Adafruit_SSD1306.h>
#include "app_state.h"

// OLED helper API used by `main.ino` and the UI task.

// Initialise the SSD1306 display at the fixed I2C address (0x3C). Returns true when the display responds.
bool display_oled_begin(Adafruit_SSD1306& display);

// Immediately render a custom message on the OLED panel.
void display_oled_show_message(Adafruit_SSD1306& display, const String& message);

// Periodically refresh the OLED with sensor telemetry derived from the AppState.
void display_oled_update(Adafruit_SSD1306& display, const AppState& state);
