#include "sensor_sys.h"

#include <WiFi.h>
#include <ArduinoJson.h>

#include "timebase.h"
#include "ui/ws_telemetry.h"

// System telemetry collector (heap, RSSI, uptime).
// This is used for basic health reporting in the web UI/telemetry stream.

void sensor_sys_update(AppState& state) {
  state.sys.uptime_ms = millis();
  state.sys.free_heap = ESP.getFreeHeap();
  state.sys.largest_block = ESP.getMaxAllocHeap();
  state.sys.wifi_rssi = WiFi.isConnected() ? WiFi.RSSI() : -127;

  static uint32_t last_emit_ms = 0;
  const uint32_t now = millis();
  if (now - last_emit_ms < 1000) {
    return;
  }
  last_emit_ms = now;

  state.sys.stamp.t_us = now_us();
  state.sys.stamp.seq += 1;

  if (!ui::telemetry_ws_has_clients()) {
    return;
  }

  JsonDocument doc;
  doc["type"] = "sys";
  doc["uptime_ms"] = state.sys.uptime_ms;
  doc["heap"] = state.sys.free_heap;
  doc["rssi"] = state.sys.wifi_rssi;
  ui::telemetry_ws_send(doc);
}
