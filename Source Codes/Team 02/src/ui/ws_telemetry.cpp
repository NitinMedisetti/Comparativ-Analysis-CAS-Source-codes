#include "ws_telemetry.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "api_config.h"
#include "core/controller_state.h"
#include "core/app_state.h"

// Telemetry WebSocket implementation:
// - Uses a FreeRTOS queue to decouple producers from the AsyncTCP sender task.
// - Never blocks the control loop; messages are dropped when under load.

namespace ui {
namespace {
constexpr size_t kTelemetryQueueDepth = 32;
constexpr size_t kTelemetryMsgMaxLen = 4096;
constexpr uint32_t kMaxSendPerTick = 6;
constexpr uint32_t kDropReportIntervalMs = 1000;

struct TelemetryMsg {
  size_t len = 0;
  char data[kTelemetryMsgMaxLen];
};

AsyncWebSocket g_ws(kEndpointTelemetryWs);
QueueHandle_t g_queue = nullptr;
uint32_t g_dropped_total = 0;
volatile uint32_t g_client_count = 0;

bool ensure_queue() {
  if (g_queue) return true;
  g_queue = xQueueCreate(kTelemetryQueueDepth, sizeof(TelemetryMsg*));
  return g_queue != nullptr;
}

static inline bool has_clients() {
  return g_client_count > 0;
}

void telemetry_report_drops_if_needed(uint32_t now_ms) {
  static uint32_t last_log_ms = 0;
  static uint32_t last_logged_total = 0;
  static uint32_t last_send_ms = 0;
  static uint32_t last_sent_total = 0;

  const UBaseType_t queued = g_queue ? uxQueueMessagesWaiting(g_queue) : 0;
  if (g_dropped_total != last_logged_total &&
      (last_log_ms == 0 || (uint32_t)(now_ms - last_log_ms) >= kDropReportIntervalMs)) {
    last_log_ms = now_ms;
    last_logged_total = g_dropped_total;
    Serial.printf("[telemetry_ws] dropped_total=%lu queued=%lu\n",
                  (unsigned long)g_dropped_total,
                  (unsigned long)queued);
  }

  if (!has_clients()) return;
  if (!g_ws.availableForWriteAll()) return;
  if (g_dropped_total == last_sent_total) return;
  if (last_send_ms != 0 && (uint32_t)(now_ms - last_send_ms) < kDropReportIntervalMs) return;
  last_send_ms = now_ms;

  char buf[128];
  const int n = snprintf(buf,
                         sizeof(buf),
                         "{\"type\":\"telemetry_stats\",\"dropped_total\":%lu,\"queued\":%lu}",
                         (unsigned long)g_dropped_total,
                         (unsigned long)queued);
  if (n > 0 && (size_t)n < sizeof(buf)) {
    g_ws.textAll(buf, (size_t)n);
    last_sent_total = g_dropped_total;
  }
}

void telemetry_ws_flush_async() {
  if (!g_queue) return;
  if (!has_clients()) return;
  if (!g_ws.availableForWriteAll()) return;

  TelemetryMsg* msg = nullptr;
  uint32_t sent = 0;
  const uint32_t now_ms = millis();

  while (sent < kMaxSendPerTick && xQueueReceive(g_queue, &msg, 0) == pdTRUE) {
    if (!msg) continue;
    if (msg->len > 0) {
      g_ws.textAll(msg->data, msg->len);
      sent++;
    }
    vPortFree(msg);
    msg = nullptr;

    if (!g_ws.availableForWriteAll()) {
      break;
    }
  }

  telemetry_report_drops_if_needed(now_ms);
}

}  // namespace

void telemetry_ws_begin(AsyncWebServer& server) {
  if (ensure_queue()) {
    g_ws.onEvent([](AsyncWebSocket* /*server*/,
                    AsyncWebSocketClient* /*client*/,
                    AwsEventType type,
                    void* /*arg*/,
                    uint8_t* /*data*/,
                    size_t /*len*/) {
      if (type == WS_EVT_CONNECT) {
        g_client_count++;
      } else if (type == WS_EVT_DISCONNECT) {
        if (g_client_count > 0) g_client_count--;
      }
      telemetry_ws_flush_async();
    });
    server.addHandler(&g_ws);
  }
}

void telemetry_ws_tick() {
  // Intentionally a no-op: AsyncWebSocket internal queues are not thread-safe when accessed
  // concurrently from the Arduino loop task and the AsyncTCP service task.
  // Flushing is triggered from inside the AsyncTCP task via a library hook.
}

bool telemetry_ws_send(const JsonDocument& doc) {
  if (!ensure_queue()) return false;
  if (!has_clients()) return true;  // drop silently when no clients are connected
  const char* type = doc["type"] | "";
  if (type && type[0] != '\0') {
    if (strcmp(type, "lidar_ui") == 0 || strcmp(type, "lidar_health") == 0 ||
        strcmp(type, "us") == 0 || strcmp(type, "map") == 0) {
      return true;
    }
  }

  TelemetryMsg* msg = static_cast<TelemetryMsg*>(pvPortMalloc(sizeof(TelemetryMsg)));
  if (!msg) return false;
  msg->len = serializeJson(doc, msg->data, sizeof(msg->data));
  if (msg->len == 0 || msg->len >= sizeof(msg->data)) {
    vPortFree(msg);
    return false;
  }
  // Never block the control loop when telemetry can't keep up.
  if (xQueueSend(g_queue, &msg, 0) != pdTRUE) {
    vPortFree(msg);
    g_dropped_total++;
    return false;
  }
  return true;
}

bool telemetry_ws_has_clients() {
  return has_clients();
}

bool telemetry_ws_send_typed(const char* type, const JsonDocument& payload) {
  if (!has_clients()) return true;
  JsonDocument doc;
  doc["type"] = type;
  doc.set(payload);
  return telemetry_ws_send(doc);
}

bool telemetry_ws_send_nav(const AppState& state) {
  if (!has_clients()) return true;
  static uint32_t last_send_ms = 0;
  const uint32_t now_ms = millis();
  if (last_send_ms != 0 && (uint32_t)(now_ms - last_send_ms) < 500) {
    return true;
  }
  last_send_ms = now_ms;

  const NavigationState& nav = state.nav;
  JsonDocument doc;
  doc["type"] = "summary";
  doc["decision"] = nav.decision;
  doc["phase"] = nav.phase;
  if (nav.target_label.length()) {
    doc["target"] = nav.target_label;
  } else if (nav.target_key.length()) {
    doc["target"] = nav.target_key;
  } else {
    doc["target"] = "";
  }
  double dist_m = NAN;
  if (!isnan(nav.segment_remaining_m)) dist_m = nav.segment_remaining_m;
  else if (!isnan(nav.distance_to_dest_m)) dist_m = nav.distance_to_dest_m;
  else if (!isnan(nav.segment_distance_m)) dist_m = nav.segment_distance_m;
  if (!isnan(dist_m)) {
    doc["dist_m"] = dist_m;
  } else {
    doc["dist_m"] = nullptr;
  }

  doc["exp_ready"] = controller_state_expander_ready();
  doc["dac_ready"] = controller_state_dac_ready();

  if (!isnan(state.gps.hdop)) {
    doc["gps_hdop"] = state.gps.hdop;
  } else {
    doc["gps_hdop"] = nullptr;
  }
  doc["gps_sats"] = state.gps.sats;
  if (!isnan(state.gps.heading_deg)) {
    doc["heading_deg"] = state.gps.heading_deg;
  } else {
    doc["heading_deg"] = nullptr;
  }

  return telemetry_ws_send(doc);
}

extern "C" void gotree2_ws_flush_hook(void) {
  telemetry_ws_flush_async();
}

}  // namespace ui
