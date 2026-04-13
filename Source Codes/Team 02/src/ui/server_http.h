#pragma once

#include <Adafruit_SSD1306.h>
#include <ESPAsyncWebServer.h>

#include "app_state.h"

namespace ui {

// HTTP server for:
// - static dashboard assets served from SPIFFS
// - a compact CBOR snapshot (`/data.cbor`)
// - simple POST endpoints for nav/actuator commands

void server_http_begin(AsyncWebServer& server, Adafruit_SSD1306& oled);
void server_http_set_state(AppState& state);
void server_http_tick(const AppState& state);

}  // namespace ui
