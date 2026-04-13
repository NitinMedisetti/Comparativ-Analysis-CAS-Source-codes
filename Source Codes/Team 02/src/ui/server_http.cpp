#include "server_http.h"

#include <ArduinoJson.h>
#include <Arduino.h>
#include <functional>
#include <SPIFFS.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "api_config.h"
#include "config/dev_params.h"
#include "core/controller_state.h"
#include "core/decision_alg.h"
#include "snapshot_builder.h"
#include "ws_telemetry.h"

// AsyncWebServer handlers for the dashboard UI.
//
// Notes:
// - The CBOR snapshot is cached for `kSnapshotIntervalMs` to reduce CPU load.
// - `g_snapshot_mutex` guards both the `AppState*` pointer and snapshot buffer.

namespace ui {
namespace {
AsyncWebServer* g_server = nullptr;
Adafruit_SSD1306* g_oled = nullptr;
AppState* g_state_ptr = nullptr;
SemaphoreHandle_t g_snapshot_mutex = nullptr;
uint8_t g_snapshot_buf[CBOR_BUFFER_SIZE];
size_t g_snapshot_len = 0;
uint32_t g_snapshot_ms = 0;
bool g_snapshot_valid = false;

bool ensure_snapshot_ready() {
  if (!g_snapshot_mutex) return false;
  if (xSemaphoreTake(g_snapshot_mutex, pdMS_TO_TICKS(25)) != pdTRUE) {
    return g_snapshot_valid;
  }
  if (!g_state_ptr) {
    g_snapshot_valid = false;
  } else {
    const uint32_t now = millis();
    if (!g_snapshot_valid || (now - g_snapshot_ms) >= kSnapshotIntervalMs) {
      g_snapshot_len = build_snapshot_cbor(*g_state_ptr, g_snapshot_buf, sizeof(g_snapshot_buf));
      g_snapshot_valid = g_snapshot_len > 0;
      g_snapshot_ms = now;
    }
  }
  xSemaphoreGive(g_snapshot_mutex);
  return g_snapshot_valid;
}

void send_json_response(AsyncWebServerRequest* request, std::function<void(JsonVariant)> builder, uint16_t code = 200) {
  JsonDocument doc;
  builder(doc.to<JsonVariant>());
  String body;
  serializeJson(doc, body);
  AsyncWebServerResponse* response = request->beginResponse(code, "application/json", body);
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);
}

static inline void assign_if_finite(JsonVariant parent, const char* key, double value) {
  if (!isnan(value) && isfinite(value)) {
    parent[key] = value;
  }
}

static bool populate_data_payload(JsonDocument& doc, const AppState& state) {
  doc.clear();

  JsonObject us = doc["us"].to<JsonObject>();
  us["front_mm"] = state.us.front_mm;
  us["right_mm"] = state.us.right_mm;
  us["back_mm"] = state.us.back_mm;
  us["left_mm"] = state.us.left_mm;
  us["d45_mm"] = state.us.d45_mm;

  JsonObject imu = doc["imu"].to<JsonObject>();
  assign_if_finite(imu, "yaw", state.imu.yaw);
  assign_if_finite(imu, "pitch", state.imu.pitch);
  assign_if_finite(imu, "roll", state.imu.roll);
  assign_if_finite(imu, "lin_accel_x", state.imu.linear_accel_x);
  assign_if_finite(imu, "lin_accel_y", state.imu.linear_accel_y);
  assign_if_finite(imu, "lin_accel_z", state.imu.linear_accel_z);
  assign_if_finite(imu, "accel_x", state.imu.accel_x);
  assign_if_finite(imu, "accel_y", state.imu.accel_y);
  assign_if_finite(imu, "accel_z", state.imu.accel_z);
  assign_if_finite(imu, "gyro_x", state.imu.gyro_x);
  assign_if_finite(imu, "gyro_y", state.imu.gyro_y);
  assign_if_finite(imu, "gyro_z", state.imu.gyro_z);
  assign_if_finite(imu, "mag_x", state.imu.mag_x);
  assign_if_finite(imu, "mag_y", state.imu.mag_y);
  assign_if_finite(imu, "mag_z", state.imu.mag_z);
  assign_if_finite(imu, "temp_c", state.imu.tempC);
  assign_if_finite(imu, "tempC", state.imu.tempC);
  imu["cal_sys"] = state.imu.cal_sys;
  imu["cal_gyro"] = state.imu.cal_gyro;
  imu["cal_accel"] = state.imu.cal_accel;
  imu["cal_mag"] = state.imu.cal_mag;
  imu["calibrated"] = state.imu.calibrated;

  JsonObject gps = doc["gps"].to<JsonObject>();
  gps["valid"] = state.gps.valid;
  assign_if_finite(gps, "lat", state.gps.lat);
  assign_if_finite(gps, "lng", state.gps.lng);
  assign_if_finite(gps, "alt_m", state.gps.alt_m);
  assign_if_finite(gps, "hdop", state.gps.hdop);
  assign_if_finite(gps, "heading_deg", state.gps.heading_deg);
  assign_if_finite(gps, "speed_kmh", state.gps.speed_kmh);
  gps["sats"] = state.gps.sats;
  gps["utc"] = state.gps.utc;
  gps["date"] = state.gps.date;
  gps["fix"] = state.gps.valid ? 1 : 0;

  JsonObject settings = doc["settings"].to<JsonObject>();
  settings["cam_url"] = state.cam_url;

  JsonObject sys = doc["sys"].to<JsonObject>();
  sys["uptime_ms"] = state.sys.uptime_ms;
  sys["heap"] = state.sys.free_heap;
  sys["rssi"] = state.sys.wifi_rssi;

  JsonObject cfg = doc["cfg"].to<JsonObject>();
  cfg["units"] = state.units;
  cfg["stabilize"] = state.stabilize;

  const NavigationState& nav_state = state.nav;
  JsonObject nav = doc["nav"].to<JsonObject>();
  const String nav_target = nav_state.target_key.length() ? nav_state.target_key : nav_state.target_label;
  nav["target"] = nav_target;
  nav["target_label"] = nav_state.target_label;
  if (!isnan(nav_state.dest_lat)) {
    nav["dest_lat"] = nav_state.dest_lat;
  }
  if (!isnan(nav_state.dest_lng)) {
    nav["dest_lng"] = nav_state.dest_lng;
  }
  nav["phase"] = nav_state.phase;
  nav["status"] = nav_state.status;
  nav["active"] = nav_state.active;
  nav["command"] = nav_state.status;
  nav["mode"] = nav_target;
  nav["decision"] = nav_state.decision;
  nav["updated_ms"] = nav_state.updated_ms;
  if (!isnan(nav_state.ref_lat)) {
    nav["ref_lat"] = nav_state.ref_lat;
  }
  if (!isnan(nav_state.ref_lng)) {
    nav["ref_lng"] = nav_state.ref_lng;
  }
  nav["ref_valid"] = nav_state.ref_valid;
  nav["ref_active"] = nav_state.ref_active;
  nav["route_seq"] = nav_state.route_seq;
  if (!isnan(nav_state.distance_to_dest_m)) {
    nav["distance_m"] = nav_state.distance_to_dest_m;
  }
  if (!isnan(nav_state.bearing_to_dest_deg)) {
    nav["bearing_deg"] = nav_state.bearing_to_dest_deg;
  }
  if (!isnan(nav_state.segment_distance_m)) {
    nav["segment_distance_m"] = nav_state.segment_distance_m;
  }
  if (!isnan(nav_state.segment_remaining_m)) {
    nav["segment_remaining_m"] = nav_state.segment_remaining_m;
  }
  if (!isnan(nav_state.segment_eta_s)) {
    nav["segment_eta_s"] = nav_state.segment_eta_s;
  }
  if (!isnan(nav_state.segment_speed_mps)) {
    nav["segment_speed_mps"] = nav_state.segment_speed_mps;
  }
  nav["segment_index"] = nav_state.segment_index;
  nav["segment_total"] = nav_state.segment_total;
  JsonObject nav_obs = nav["obstacles"].to<JsonObject>();
  nav_obs["front"] = nav_state.obstacles.front_blocked ? "blocked" : "free";
  nav_obs["left"] = nav_state.obstacles.left_blocked ? "blocked" : "free";
  nav_obs["right"] = nav_state.obstacles.right_blocked ? "blocked" : "free";
  nav_obs["rear"] = nav_state.obstacles.rear_blocked ? "blocked" : "free";
  nav_obs["hard_stop"] = nav_state.obstacles.hard_stop;
  nav_obs["possible"] = nav_state.obstacles.possible_obstacles;

  return true;
}

static bool build_data_payload(JsonDocument& doc) {
  if (!g_state_ptr || !g_snapshot_mutex) {
    return false;
  }
  if (xSemaphoreTake(g_snapshot_mutex, pdMS_TO_TICKS(25)) != pdTRUE) {
    return false;
  }
  const AppState& state = *g_state_ptr;
  const bool built = populate_data_payload(doc, state);
  xSemaphoreGive(g_snapshot_mutex);
  return built;
}

void handle_nav_command(AsyncWebServerRequest* request, const char* action) {
  nav::handle_nav_command(action);
  send_json_response(request, [action](JsonVariant json) {
    json["queued"] = true;
    json["action"] = action;
  });
}

void handle_nav_command_if_ready(AsyncWebServerRequest* request, const char* action) {
  if (!controller_state_active()) {
    send_json_response(request, [action](JsonVariant json) {
      json["queued"] = false;
      json["action"] = action;
      json["error"] = "controller_inactive";
    }, 409);
    return;
  }
  const NavigationState& nav_state = nav::nav_state();
  if (!nav_state.ready_for_goto) {
    send_json_response(request, [action](JsonVariant json) {
      json["queued"] = false;
      json["action"] = action;
      json["error"] = "nav_not_ready";
    }, 409);
    return;
  }
  handle_nav_command(request, action);
}

void handle_actuator_command(AsyncWebServerRequest* request,
                            const char* action,
                            bool queued = true,
                            const char* error = nullptr,
                            uint16_t code = 200) {
  send_json_response(request, [action, queued, error](JsonVariant json) {
    json["queued"] = queued;
    json["action"] = action;
    if (!queued && error) {
      json["error"] = error;
    }
  }, code);
}

void handle_config_get(AsyncWebServerRequest* request) {
  String cam = "";
  String units = "mm";
  bool stabilize = false;
  if (g_state_ptr) {
    cam = g_state_ptr->cam_url;
    units = g_state_ptr->units.length() ? g_state_ptr->units : units;
    stabilize = g_state_ptr->stabilize;
  }
  send_json_response(request, [cam, units, stabilize](JsonVariant json) {
    json["cam_url"] = cam;
    json["units"] = units;
    json["stabilize"] = stabilize;
  });
}

}  // namespace

void server_http_begin(AsyncWebServer& server, Adafruit_SSD1306& oled) {
  g_server = &server;
  g_oled = &oled;
  if (!g_snapshot_mutex) {
    g_snapshot_mutex = xSemaphoreCreateMutex();
  }
  SPIFFS.begin(true);

  server.on(kEndpointDataCbor, HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!g_state_ptr || !ensure_snapshot_ready()) {
      request->send(503, "text/plain", "snapshot unavailable");
      return;
    }
    AsyncWebServerResponse* response =
        request->beginResponse_P(200, "application/cbor", g_snapshot_buf, g_snapshot_len);
    response->addHeader("Cache-Control", "no-cache");
    request->send(response);
  });

  server.on(kEndpointNavStop, HTTP_POST, [](AsyncWebServerRequest* request) {
    handle_nav_command(request, "stop");
  });
  server.on(kEndpointNavGotoPost, HTTP_POST, [](AsyncWebServerRequest* request) {
    handle_nav_command_if_ready(request, "goto_post");
  });
  server.on(kEndpointNavGotoH, HTTP_POST, [](AsyncWebServerRequest* request) {
    handle_nav_command_if_ready(request, "goto_h");
  });
  server.on(kEndpointActuatorInitialCheckup, HTTP_POST, [](AsyncWebServerRequest* request) {
    const bool expander_ready = controller_state_expander_ready();
    const bool dac_ready = controller_state_dac_ready();
    const bool bypass = !expander_ready || !dac_ready;

    const bool ok = true;

    if (ok) {
      controller_state_request_checkup();
      nav::handle_nav_command("start_system");
    }

    send_json_response(request, [ok, expander_ready, dac_ready, bypass](JsonVariant json) {
      json["queued"] = ok; 
      json["action"] = "initial_checkup";
      json["expander_ready"] = expander_ready;
      json["dac_ready"] = dac_ready;
      json["bypass"] = bypass;
    }, 200);
  });
  server.on(kEndpointActuatorStop, HTTP_POST, [](AsyncWebServerRequest* request) {
    const bool ok = controller_state_dac_ready() || controller_state_test_kit();
    if (ok) {
      controller_state_request_stop();
    }
    handle_actuator_command(request, "stop", ok, ok ? nullptr : "dac_unavailable");
  });
  server.on(kEndpointConfig, HTTP_GET, [](AsyncWebServerRequest* request) {
    handle_config_get(request);
  });

  telemetry_ws_begin(server);

  // Keep static file serving last so it doesn't steal requests meant for APIs or WebSockets.
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
}

void server_http_set_state(AppState& state) {
  g_state_ptr = &state;
  g_snapshot_valid = false;
}

void server_http_tick(const AppState& state) {
  (void)state;
  // Intentionally no OLED updates here: the main loop already throttles display
  // refreshes to protect the sensor/nav/control loop from long I2C transactions.
}

}  // namespace ui
