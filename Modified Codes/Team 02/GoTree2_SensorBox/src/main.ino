// GoTree2 SensorBox firmware entry point.
//
// High-level structure:
// - `Task_Sensors`: polls sensors and updates `AppState`.
// - `Task_NavControl`: runs the navigation/decision loop.
// - `Task_TelemetryUI`: serves the web UI + OLED + telemetry.
// Shared resources are protected with FreeRTOS mutexes (`g_state_mutex`, `g_i2c_mutex`).

#include <Adafruit_SSD1306.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "config/dev_params.h"
#include "core/app_state.h"
#include "drivers/display_oled.h"
#include "actuators/Controller_solid.h"
#include "core/controller_state.h"
#include "core/decision_alg.h"
#include "sensors/sensor_gps.h"
#include "sensors/sensor_imu.h"
#include "sensors/sensor_lidar.h"
#include "sensors/sensor_sys.h"
#include "sensors/sensor_us.h"
#include "ui/server_http.h"
#include "ui/ws_telemetry.h"

namespace {

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

constexpr int US45_TRIG = 23;
constexpr int US45_ECHO = 26;
constexpr int USF_TRIG = 32;
constexpr int USF_ECHO = 25;
constexpr int USB_TRIG = 5;
constexpr int USB_ECHO = 14;
constexpr int USL_TRIG = 4;
constexpr int USL_ECHO = 18;
constexpr int USR_TRIG = 13;
constexpr int USR_ECHO = 19;

constexpr int LIDAR_RX = 27;
constexpr int LIDAR_TX = 33;
constexpr int GPS_RX = 16;
constexpr int GPS_TX = 17;

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 40;
constexpr int OLED_RESET = -1;
constexpr uint32_t kDisplayIntervalMs = 100;
constexpr uint32_t kNavTaskIntervalMs = 50;
constexpr uint32_t kSensorTaskDelayMs = 5;

AppStateBuffer g_state;
AsyncWebServer g_server(80);
Adafruit_SSD1306 g_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TinyGPSPlus g_gps;
AppState g_ui_state;
SemaphoreHandle_t g_i2c_mutex = nullptr;
SemaphoreHandle_t g_state_mutex = nullptr;
TaskHandle_t g_nav_task = nullptr;
TaskHandle_t g_sensor_task = nullptr;
TaskHandle_t g_ui_task = nullptr;

const SensorUSPins kUsPins = {
  USF_TRIG, USF_ECHO,
  USB_TRIG, USB_ECHO,
  USL_TRIG, USL_ECHO,
  USR_TRIG, USR_ECHO,
  US45_TRIG, US45_ECHO
};

// Safety: stop the robot if the nav task fails to tick for too long.
constexpr uint32_t kNavSafetyTimeoutMs = 1500;
constexpr uint32_t kNavTelemetryMinIntervalMs = 500;
constexpr uint32_t kControllerTelemetryMinIntervalMs = 2000;

portMUX_TYPE g_controller_mux = portMUX_INITIALIZER_UNLOCKED;
bool g_expander_ready = false;
bool g_dac_ready = false;
bool g_controller_active = false;
bool g_test_kit_mode = false;
bool g_checkup_requested = false;
bool g_checkup_in_progress = false;
bool g_stop_requested = false;
TaskHandle_t g_checkup_task = nullptr;

void Task_NavControl(void*);
void Task_Sensors(void*);
void Task_TelemetryUI(void*);

bool get_flag(bool& flag) {
  portENTER_CRITICAL(&g_controller_mux);
  const bool value = flag;
  portEXIT_CRITICAL(&g_controller_mux);
  return value;
}

void set_flag(bool& flag, bool value) {
  portENTER_CRITICAL(&g_controller_mux);
  flag = value;
  portEXIT_CRITICAL(&g_controller_mux);
}

void checkup_task_main(void*) {
  if (g_i2c_mutex) {
    xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    xSemaphoreGive(g_i2c_mutex);
  }

  if (g_nav_task) vTaskSuspend(g_nav_task);
  if (g_sensor_task) vTaskSuspend(g_sensor_task);

  initialcheckuproutine();

  portENTER_CRITICAL(&g_controller_mux);
  g_checkup_in_progress = false;
  g_checkup_requested = false;
  g_controller_active = true;
  g_checkup_task = nullptr;
  portEXIT_CRITICAL(&g_controller_mux);

  if (g_nav_task) vTaskResume(g_nav_task);
  if (g_sensor_task) vTaskResume(g_sensor_task);

  vTaskDelete(nullptr);
}

void tick_controller_checkup() {
  bool should_start = false;

  portENTER_CRITICAL(&g_controller_mux);
  const bool prerequisites_ok = g_expander_ready && g_dac_ready;
  if (g_checkup_requested && g_controller_active) {
    g_checkup_requested = false;
  }
  const bool can_start = g_checkup_requested && !g_checkup_in_progress && !g_controller_active &&
                         (g_checkup_task == nullptr);

  if (can_start && prerequisites_ok) {
    g_checkup_in_progress = true;
    should_start = true;
  } else if (can_start && !prerequisites_ok) {
    g_checkup_requested = false;
    g_controller_active = true;
    g_test_kit_mode = true;
  }
  portEXIT_CRITICAL(&g_controller_mux);

  if (!should_start) {
    return;
  }

  const BaseType_t ok = xTaskCreatePinnedToCore(
      checkup_task_main,
      "controller_checkup",
      8192,
      nullptr,
      2,
      &g_checkup_task,
      1);

  if (ok != pdPASS) {
    portENTER_CRITICAL(&g_controller_mux);
    g_checkup_in_progress = false;
    g_checkup_requested = false;
    g_checkup_task = nullptr;
    portEXIT_CRITICAL(&g_controller_mux);
  }
}

const char* reset_reason_name(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN: return "unknown";
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "other";
  }
}

void log_reset_reason() {
  const esp_reset_reason_t reason = esp_reset_reason();
  Serial.printf("[Boot] Reset reason: %d (%s)\n", (int)reason, reset_reason_name(reason));
}

void send_nav_telemetry(const AppState& state, uint32_t now_ms) {
  const NavigationState& nav = state.nav;
  static uint32_t last_send_ms = 0;
  static uint8_t last_path_len = 0;
  static uint8_t last_path_ids[32] = {};
  static uint8_t last_path_idx = 0;
  static uint8_t last_route_seq = 0;
  static String last_decision;
  static String last_phase;
  static bool last_ref_valid = false;
  static bool last_ref_active = false;
  static bool last_system_started = false;
  static bool last_heading_ready = false;
  static bool last_gps_ready = false;
  static bool last_ready_for_goto = false;

  const bool len_changed = nav.path_len != last_path_len;
  bool ids_changed = len_changed;
  if (!ids_changed) {
    for (uint8_t i = 0; i < nav.path_len && i < sizeof(last_path_ids); ++i) {
      if (nav.path_ids[i] != last_path_ids[i]) {
        ids_changed = true;
        break;
      }
    }
  }

  const bool meta_changed = (nav.current_path_idx != last_path_idx) || (nav.route_seq != last_route_seq) ||
                            (nav.decision != last_decision) || (nav.phase != last_phase) ||
                            (nav.ref_valid != last_ref_valid) || (nav.ref_active != last_ref_active) ||
                            (nav.system_started != last_system_started) ||
                            (nav.heading_ready != last_heading_ready) ||
                            (nav.gps_ready != last_gps_ready) ||
                            (nav.ready_for_goto != last_ready_for_goto);

  if (!ids_changed && !meta_changed && (now_ms - last_send_ms) < kNavTelemetryMinIntervalMs) {
    return;
  }

  last_send_ms = now_ms;
  last_path_len = nav.path_len;
  for (uint8_t i = 0; i < sizeof(last_path_ids); ++i) {
    last_path_ids[i] = (i < nav.path_len) ? nav.path_ids[i] : 0;
  }
  last_path_idx = nav.current_path_idx;
  last_route_seq = nav.route_seq;
  last_decision = nav.decision;
  last_phase = nav.phase;
  last_ref_valid = nav.ref_valid;
  last_ref_active = nav.ref_active;
  last_system_started = nav.system_started;
  last_heading_ready = nav.heading_ready;
  last_gps_ready = nav.gps_ready;
  last_ready_for_goto = nav.ready_for_goto;

  ui::telemetry_ws_send_nav(state);
}

// --- GÜVENLİK VE HAREKET UYGULAYICI ---
void send_controller_telemetry(uint32_t now_ms) {
  static uint32_t last_send_ms = 0;
  static bool last_expander_ready = false;
  static bool last_dac_ready = false;
  static bool last_active = false;

  const bool expander_ready = ::controller_state_expander_ready();
  const bool dac_ready = ::controller_state_dac_ready();
  const bool active = ::controller_state_active();

  const bool changed = (expander_ready != last_expander_ready) ||
                       (dac_ready != last_dac_ready) ||
                       (active != last_active);
  if (!changed && (now_ms - last_send_ms) < kControllerTelemetryMinIntervalMs) {
    return;
  }

  if (!ui::telemetry_ws_has_clients()) {
    return;
  }

  last_send_ms = now_ms;
  last_expander_ready = expander_ready;
  last_dac_ready = dac_ready;
  last_active = active;

  JsonDocument doc;
  doc["type"] = "controller";
  doc["expander_ready"] = expander_ready;
  doc["dac_ready"] = dac_ready;
  doc["active"] = active;
  ui::telemetry_ws_send(doc);
}

void apply_nav_decision(const NavigationState& nav, uint32_t now_ms) {
  enum class MotionKind : uint8_t {
    Stop = 0,
    Forward,
    Backward,
    RotateLeft,
    RotateRight,
    StepLeft,
    StepRight,
    Unknown,
  };

  auto parse_motion = [](const String& decision) -> MotionKind {
    if (decision == "FORWARD") return MotionKind::Forward;
    if (decision == "BACKWARD") return MotionKind::Backward;
    if (decision == "ROTATE_LEFT" || decision == "LEFT") return MotionKind::RotateLeft;
    if (decision == "ROTATE_RIGHT" || decision == "RIGHT") return MotionKind::RotateRight;
    if (decision == "STEP_LEFT") return MotionKind::StepLeft;
    if (decision == "STEP_RIGHT") return MotionKind::StepRight;
    if (decision == "STOP") return MotionKind::Stop;
    return MotionKind::Stop;
  };

  static MotionKind last_motion = MotionKind::Unknown;
  static int last_speed = -1;
  static String last_decision = "";

  if ((now_ms - nav.updated_ms) > kNavSafetyTimeoutMs) {
    if (last_motion != MotionKind::Stop) {
      if (::controller_state_dac_ready() || ::controller_state_test_kit()) {
        stop();
      }
      last_motion = MotionKind::Stop;
      last_speed = 0;
      Serial.println("[Safety] Nav watchdog triggered: STOP");
    }
    return;
  }

  const MotionKind motion = nav.system_started ? parse_motion(nav.decision) : MotionKind::Stop;
  const int effective_speed = (motion == MotionKind::Stop) ? 0 : SPEED_LIMIT;
  const bool allow_actuation = ::controller_state_dac_ready() || ::controller_state_test_kit();

  if ((nav.decision == last_decision) && (motion == last_motion) && (effective_speed == last_speed)) {
    return;
  }
  if (!allow_actuation) {
    return;
  }

  if (motion == MotionKind::Forward) {
    forward(effective_speed);
  } else if (motion == MotionKind::Backward) {
    backward(effective_speed);
  } else if (motion == MotionKind::RotateLeft) {
    rotateLeft(effective_speed);
  } else if (motion == MotionKind::RotateRight) {
    rotateRight(effective_speed);
  } else if (motion == MotionKind::StepLeft) {
    stepLeft(effective_speed);
  } else if (motion == MotionKind::StepRight) {
    stepRight(effective_speed);
  } else {
    stop(); 
  }
  last_motion = motion;
  last_speed = effective_speed;
  last_decision = nav.decision;
}

IPAddress parse_ip(const char* str, uint8_t fallback3 = 0) {
  IPAddress ip;
  if (!ip.fromString(str)) {
    ip = IPAddress(192, 168, 4, fallback3);
  }
  return ip;
}

void init_wifi_softap() {
  const IPAddress ap_ip = parse_ip(SOFTAP_IP, 1);
  const IPAddress gateway = ap_ip;
  const IPAddress subnet(255, 255, 255, 0);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ap_ip, gateway, subnet);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
}

void configure_camera_defaults(AppState& state) {
  state.cam_url = "";
  state.cam_status = "Camera support removed";
}

void copy_ui_state(AppState& dst, const AppState& src) {
  dst.gps = src.gps;
  dst.imu = src.imu;
  dst.us = src.us;
  dst.nav = src.nav;
  dst.oled_label = src.oled_label;
}

void Task_NavControl(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  NavigationState nav_snapshot = {};

  while (true) {
    tick_controller_checkup();
    if (::controller_state_checkup_in_progress()) {
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kNavTaskIntervalMs));
      continue;
    }

    if (!::controller_state_checkup_in_progress() &&
        ::controller_state_consume_stop_request() &&
        (::controller_state_dac_ready() || ::controller_state_test_kit())) {
      if (g_i2c_mutex && xSemaphoreTake(g_i2c_mutex, portMAX_DELAY) == pdTRUE) {
        stop();
        xSemaphoreGive(g_i2c_mutex);
      }
    }

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
      AppState& writable = g_state.writable();
      nav::nav_tick(writable, writable);
      nav_snapshot = writable.nav;
      xSemaphoreGive(g_state_mutex);
    }

    if (::controller_state_checkup_in_progress()) {
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kNavTaskIntervalMs));
      continue;
    }

    uint32_t now = millis();
    if (!::controller_state_active()) {
      if (::controller_state_dac_ready()) {
        if (g_i2c_mutex && xSemaphoreTake(g_i2c_mutex, portMAX_DELAY) == pdTRUE) {
          stop();
          xSemaphoreGive(g_i2c_mutex);
        }
      }
    } else {
      if (g_i2c_mutex && xSemaphoreTake(g_i2c_mutex, portMAX_DELAY) == pdTRUE) {
        apply_nav_decision(nav_snapshot, now);
        xSemaphoreGive(g_i2c_mutex);
      }
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kNavTaskIntervalMs));
  }
}

void Task_Sensors(void*) {
  const TickType_t delay_ticks = pdMS_TO_TICKS(kSensorTaskDelayMs);

  while (true) {
    if (::controller_state_checkup_in_progress()) {
      vTaskDelay(delay_ticks);
      continue;
    }

    sensor_gps_poll(g_gps);

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
      AppState& writable = g_state.writable();
      sensor_gps_update_state(g_gps, writable);
      xSemaphoreGive(g_state_mutex);
    }

    if (g_i2c_mutex && xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        AppState& writable = g_state.writable();
        sensor_imu_update(writable);
        xSemaphoreGive(g_state_mutex);
      }
      xSemaphoreGive(g_i2c_mutex);
    }

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
      AppState& writable = g_state.writable();
      sensor_us_update(writable);
      sensor_lidar_poll(writable);
      sensor_sys_update(writable);
      xSemaphoreGive(g_state_mutex);
    }

    vTaskDelay(delay_ticks);
  }
}

void Task_TelemetryUI(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t last_snapshot_ms = 0;
  const uint32_t snapshot_interval = SNAPSHOT_HZ ? (1000 / SNAPSHOT_HZ) : 1000;

  while (true) {
    const uint32_t now = millis();

    if (!::controller_state_checkup_in_progress()) {
      if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        copy_ui_state(g_ui_state, g_state.writable());
        xSemaphoreGive(g_state_mutex);

        if (g_i2c_mutex && xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          display_oled_update(g_display, g_ui_state);
          xSemaphoreGive(g_i2c_mutex);
        }
      }
    }

    send_nav_telemetry(g_ui_state, now);
    send_controller_telemetry(now);
    ui::telemetry_ws_tick();

    if (now - last_snapshot_ms >= snapshot_interval) {
      last_snapshot_ms = now;
      if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        ui::server_http_set_state(g_state.writable());
        g_state.publish();
        xSemaphoreGive(g_state_mutex);
      }
    }

    ui::server_http_tick(g_state.readable());
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kDisplayIntervalMs));
  }
}

}  // namespace

bool controller_state_expander_ready() { return get_flag(g_expander_ready); }
bool controller_state_dac_ready() { return get_flag(g_dac_ready); }
bool controller_state_active() { return get_flag(g_controller_active); }
bool controller_state_test_kit() { return get_flag(g_test_kit_mode); }
bool controller_state_checkup_in_progress() { return get_flag(g_checkup_in_progress); }

void controller_state_request_checkup() { set_flag(g_checkup_requested, true); }
void controller_state_request_stop() { set_flag(g_stop_requested, true); }

bool controller_state_consume_stop_request() {
  portENTER_CRITICAL(&g_controller_mux);
  const bool value = g_stop_requested;
  g_stop_requested = false;
  portEXIT_CRITICAL(&g_controller_mux);
  return value;
}

void setup() {
  Serial.begin(115200);
  log_reset_reason();

  g_i2c_mutex = xSemaphoreCreateMutex();
  g_state_mutex = xSemaphoreCreateMutex();

  SPIFFS.begin(true);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(OLED_I2C_FREQ); 

  Wire.setTimeOut(50);

  // Controller Init (retry with slower I2C if needed)
  bool exp_ok = initExpMod();
  bool dac_ok = initDAC();
  if (!exp_ok || !dac_ok) {
    Wire.setClock(I2C_FALLBACK_FREQ);
    if (!exp_ok) exp_ok = initExpMod();
    if (!dac_ok) dac_ok = initDAC();
  }
  const bool is_robot = exp_ok && dac_ok;
  set_flag(g_expander_ready, exp_ok);
  set_flag(g_dac_ready, dac_ok);
  set_flag(g_test_kit_mode, !is_robot);
  set_flag(g_controller_active, !is_robot);
  set_flag(g_checkup_requested, false);
  set_flag(g_checkup_in_progress, false);
  set_flag(g_stop_requested, false);
  if (is_robot) {
    Serial.println("Controller hardware ready.");
  } else {
    Serial.println("Test kit mode: hardware not detected.");
  }

  display_oled_begin(g_display);
  display_oled_show_message(g_display, "Booting...");

  // Sensors
  sensor_imu_begin();
  sensor_us_begin(kUsPins);
  sensor_lidar_begin(LIDAR_RX, LIDAR_TX);
  sensor_gps_begin(GPS_RX, GPS_TX);

  // AppState Init
  AppState& readable = g_state.readable();
  AppState& writable = g_state.writable();
  app_state_reset(readable);
  app_state_reset(writable);
  app_state_reset(g_ui_state);
  
  configure_camera_defaults(writable);
  writable.oled_label = "System Boot";
  
  readable = writable;
  g_state.publish();
  
  ui::server_http_set_state(g_state.readable());

  init_wifi_softap();

  ui::server_http_begin(g_server, g_display);
  g_server.begin();

  xTaskCreatePinnedToCore(Task_NavControl, "Task_NavControl", 8192, nullptr, 4, &g_nav_task, 1);
  xTaskCreatePinnedToCore(Task_Sensors, "Task_Sensors", 8192, nullptr, 3, &g_sensor_task, 1);
  xTaskCreatePinnedToCore(Task_TelemetryUI, "Task_TelemetryUI", 8192, nullptr, 1, &g_ui_task, 0);
}

void loop() {
  vTaskDelete(nullptr);
}
