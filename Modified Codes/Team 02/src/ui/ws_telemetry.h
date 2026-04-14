#pragma once

#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>

struct AppState;

namespace ui {

// WebSocket telemetry transport.
//
// Messages are queued from producer code (sensors/nav), and flushed from inside
// the AsyncTCP task via `gotree2_ws_flush_hook()` to avoid cross-thread access to
// AsyncWebSocket internals.

// Initialize WebSocket endpoint and sender task.
void telemetry_ws_begin(AsyncWebServer& server);

// Flush queued telemetry messages (call from main loop).
void telemetry_ws_tick();

// Fast check to avoid building JSON documents when no clients are connected.
bool telemetry_ws_has_clients();

// Enqueue a JSON message that already contains a "type" field.
bool telemetry_ws_send(const JsonDocument& doc);

// Convenience helper: wraps the given payload under { "type": <type>, ...payload }.
bool telemetry_ws_send_typed(const char* type, const JsonDocument& payload);

// Serialize + enqueue a nav message (type="nav").
bool telemetry_ws_send_nav(const AppState& state);

}  // namespace ui
