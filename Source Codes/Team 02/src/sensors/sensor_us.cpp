#include "sensor_us.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_timer.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config/dev_params.h"
#include "timebase.h"
#include "ui/ws_telemetry.h"

// Ultrasonic range measurement (HC-SR04 style).
//
// Implementation overview:
// - A background FreeRTOS task triggers active sensors in a round-robin.
// - Echo pulse capture is interrupt-driven to avoid blocking the CPU.
// - `sensor_us_update()` just copies the latest cached readings into `AppState`.

static SensorUSPins us_pins;
static bool pins_initialised = false;
static constexpr uint32_t kTelemetryIntervalMs = 1000;
static constexpr uint32_t kUsTimeoutUs = 17500;  // ~3 m range round trip with margin
static uint8_t s_active_mask = (1u << 0) | (1u << 2) | (1u << 3);  // F + L + R (default)

static portMUX_TYPE s_us_mux = portMUX_INITIALIZER_UNLOCKED;

static USBlock s_us_cache;
static uint8_t s_last_updated_idx = 0xFF;
static float s_sound_speed_mm_per_us = 0.343f;  // ~20C
static TaskHandle_t s_us_task_handle = nullptr;
static uint8_t s_irq_mask = 0;  // which echo interrupts are attached/enabled

namespace {

constexpr uint8_t kUsSensorCount = 5;

struct EchoCapture {
  volatile uint32_t rise_us = 0;
  volatile uint32_t fall_us = 0;
  // 0 = idle, 1 = waiting for rising edge, 2 = waiting for falling edge, 3 = done
  volatile uint8_t stage = 0;
};

EchoCapture s_echo[kUsSensorCount];

static inline uint8_t idx_from_arg(void* arg) {
  return static_cast<uint8_t>(reinterpret_cast<uintptr_t>(arg));
}

void IRAM_ATTR echo_isr(void* arg) {
  const uint8_t idx = idx_from_arg(arg);
  if (idx >= kUsSensorCount) return;

  const int echo_pin = [&]() -> int {
    switch (idx) {
      case 0: return us_pins.echo_front;
      case 1: return us_pins.echo_back;
      case 2: return us_pins.echo_left;
      case 3: return us_pins.echo_right;
      case 4: return us_pins.echo_diag45;
      default: return -1;
    }
  }();
  if (echo_pin < 0) return;

  const uint8_t stage = s_echo[idx].stage;
  if (stage == 0 || stage == 3) return;  // not expecting anything

  const int level = digitalRead(echo_pin);
  const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());

  if (level) {
    if (stage == 1) {
      s_echo[idx].rise_us = now_us;
      s_echo[idx].stage = 2;
    }
    return;
  }

  if (stage == 2) {
    s_echo[idx].fall_us = now_us;
    s_echo[idx].stage = 3;

    if (s_us_task_handle) {
      BaseType_t woken = pdFALSE;
      xTaskNotifyFromISR(s_us_task_handle, (1u << idx), eSetBits, &woken);
      if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
      }
    }
  }
}

static inline long duration_to_mm(uint32_t duration_us, float speed_mm_per_us) {
  if (duration_us == 0) return 0;
  const float mm = (static_cast<float>(duration_us) * speed_mm_per_us) / 2.0f;
  return (long)mm;
}

static inline int us_trig_for_idx(uint8_t idx) {
  switch (idx) {
    case 0: return us_pins.trig_front;
    case 1: return us_pins.trig_back;
    case 2: return us_pins.trig_left;
    case 3: return us_pins.trig_right;
    case 4: return us_pins.trig_diag45;
    default: return -1;
  }
}

static inline int us_echo_for_idx(uint8_t idx) {
  switch (idx) {
    case 0: return us_pins.echo_front;
    case 1: return us_pins.echo_back;
    case 2: return us_pins.echo_left;
    case 3: return us_pins.echo_right;
    case 4: return us_pins.echo_diag45;
    default: return -1;
  }
}

static void enable_echo_irq(uint8_t idx, bool enable) {
  const int echo_pin = us_echo_for_idx(idx);
  if (echo_pin < 0) return;
  if (enable) {
    s_echo[idx] = {};
    attachInterruptArg(echo_pin, echo_isr, reinterpret_cast<void*>(static_cast<uintptr_t>(idx)), CHANGE);
  } else {
    detachInterrupt(echo_pin);
    s_echo[idx].stage = 0;
    s_echo[idx].rise_us = 0;
    s_echo[idx].fall_us = 0;
  }
}

static void update_echo_interrupts(uint8_t desired_mask) {
  for (uint8_t idx = 0; idx < kUsSensorCount; ++idx) {
    const uint8_t bit = (1u << idx);
    const bool want = (desired_mask & bit) != 0;
    const bool have = (s_irq_mask & bit) != 0;
    if (want == have) continue;
    enable_echo_irq(idx, want);
    if (want) s_irq_mask |= bit;
    else s_irq_mask &= (uint8_t)~bit;
  }
}

static void attach_echo_interrupts(uint8_t initial_mask) {
  for (uint8_t idx = 0; idx < kUsSensorCount; ++idx) {
    // Ensure a clean slate; the actual attach is driven by the mask.
    s_echo[idx] = {};
  }
  s_irq_mask = 0;
  update_echo_interrupts(initial_mask);
}

void us_measure_task(void* /*unused*/);

}  // namespace

void sensor_us_begin(const SensorUSPins& pins){
  us_pins = pins;

  const int triggers[] = {pins.trig_front, pins.trig_back, pins.trig_left, pins.trig_right, pins.trig_diag45};
  const int echos[] = {pins.echo_front, pins.echo_back, pins.echo_left, pins.echo_right, pins.echo_diag45};

  for(int i=0; i<5; i++){
      pinMode(triggers[i], OUTPUT);
      pinMode(echos[i], INPUT);
      digitalWrite(triggers[i], LOW);
  }

  portENTER_CRITICAL(&s_us_mux);
  s_us_cache = {};
  s_last_updated_idx = 0xFF;
  portEXIT_CRITICAL(&s_us_mux);

  pins_initialised = true;

  if (!s_us_task_handle) {
    // Run ultrasonic measurement away from the WiFi/AsyncTCP stack (typically core 0).
    // Keep priority below the Arduino loop task so controller/OLED/nav stay responsive.
    xTaskCreatePinnedToCore(us_measure_task, "us_measure_task", 2048, nullptr, 0, &s_us_task_handle, 1);
  }

  attach_echo_interrupts(s_active_mask);
}

void sensor_us_set_active(bool f, bool b, bool l, bool r, bool d) {
  uint8_t mask = 0;
  if (f) mask |= (1u << 0);
  if (b) mask |= (1u << 1);
  if (l) mask |= (1u << 2);
  if (r) mask |= (1u << 3);
  if (d) mask |= (1u << 4);

  uint8_t prev = 0;
  portENTER_CRITICAL(&s_us_mux);
  prev = s_active_mask;
  s_active_mask = mask;

  // When disabling a sensor, clear its cached distance so decision_alg won't act on stale data.
  const uint8_t disabled = (uint8_t)(prev & (uint8_t)~mask);
  if (disabled & (1u << 0)) s_us_cache.front_mm = 0;
  if (disabled & (1u << 1)) s_us_cache.back_mm = 0;
  if (disabled & (1u << 2)) s_us_cache.left_mm = 0;
  if (disabled & (1u << 3)) s_us_cache.right_mm = 0;
  if (disabled & (1u << 4)) s_us_cache.d45_mm = 0;
  portEXIT_CRITICAL(&s_us_mux);

  // Enable only the requested echo IRQs to reduce ISR load/noise processing.
  if (pins_initialised) {
    update_echo_interrupts(mask);
  }
}

static const char* us_index_key(uint8_t idx) {
  switch (idx) {
    case 0: return "front";
    case 1: return "back";
    case 2: return "left";
    case 3: return "right";
    case 4: return "diag45";
    default: return "unknown";
  }
}

namespace {

void us_measure_task(void* /*unused*/) {
  for (;;) {
    if (!pins_initialised) {
      vTaskDelay(50 / portTICK_PERIOD_MS);
      continue;
    }

    uint8_t active_mask = 0;
    float speed_mm_per_us = 0.343f;
    portENTER_CRITICAL(&s_us_mux);
    active_mask = s_active_mask;
    speed_mm_per_us = s_sound_speed_mm_per_us;
    portEXIT_CRITICAL(&s_us_mux);

    bool any_active = false;

    uint8_t active_indices[kUsSensorCount];
    uint8_t active_count = 0;
    for (uint8_t idx = 0; idx < kUsSensorCount; ++idx) {
      if ((active_mask & (1u << idx)) == 0) continue;
      active_indices[active_count++] = idx;
    }

    for (uint8_t i = 0; i < active_count; ++i) {
      const uint8_t idx = active_indices[i];

      any_active = true;
      const int trig = us_trig_for_idx(idx);
      const int echo = us_echo_for_idx(idx);
      long mm = 0;
      uint64_t t_us = now_us();

      if (trig >= 0 && echo >= 0) {
        // Clear any stale notification for this channel before triggering.
        uint32_t ignored = 0;
        xTaskNotifyWait((1u << idx), (1u << idx), &ignored, 0);

        // Arm capture.
        s_echo[idx].rise_us = 0;
        s_echo[idx].fall_us = 0;
        s_echo[idx].stage = 1;

        // Trigger pulse.
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);

        // Wait for echo completion (interrupt-driven) without busy-waiting the CPU.
        uint32_t notified = 0;
        const uint32_t timeout_ms = (kUsTimeoutUs / 1000u) + 2u;
        const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
        const BaseType_t ok = xTaskNotifyWait(0, (1u << idx), &notified, timeout_ticks);

        if (ok == pdTRUE && (notified & (1u << idx)) != 0 && s_echo[idx].stage == 3) {
          const uint32_t rise_us = s_echo[idx].rise_us;
          const uint32_t fall_us = s_echo[idx].fall_us;
          const uint32_t duration_us = (fall_us >= rise_us) ? (fall_us - rise_us) : 0;
          mm = duration_to_mm(duration_us, speed_mm_per_us);
          t_us = now_us();
        } else {
          mm = 0;
        }

        s_echo[idx].stage = 0;
      }

      portENTER_CRITICAL(&s_us_mux);
      switch (idx) {
        case 0: s_us_cache.front_mm = mm; break;
        case 1: s_us_cache.back_mm = mm; break;
        case 2: s_us_cache.left_mm = mm; break;
        case 3: s_us_cache.right_mm = mm; break;
        case 4: s_us_cache.d45_mm = mm; break;
      }
      s_us_cache.stamp.t_us = t_us;
      s_us_cache.stamp.seq += 1;
      s_last_updated_idx = idx;
      portEXIT_CRITICAL(&s_us_mux);

      // Let echoes die out before moving to the next sensor to reduce crosstalk.
      const uint32_t delay_ms = (active_count <= 1) ? 5u : 15u;
      vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }

    if (!any_active) {
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}

}  // namespace

void sensor_us_update(AppState& state){
  if (!pins_initialised){
    portENTER_CRITICAL(&s_us_mux);
    s_us_cache = {};
    s_last_updated_idx = 0xFF;
    portEXIT_CRITICAL(&s_us_mux);
    state.us.front_mm = state.us.back_mm = state.us.left_mm = state.us.right_mm = state.us.d45_mm = 0;
    state.us.stamp = {};
    return;
  }

  const uint32_t now = millis();

  float temp_c = 20.0f;
  if (isfinite(state.imu.tempC)) {
    temp_c = state.imu.tempC;
  } else if (state.gps.valid && !isnan(state.gps.temp_c)) {
    temp_c = state.gps.temp_c;
  }

  const float speed_m_s = 331.3f + (0.606f * temp_c);
  const float speed_mm_per_us = speed_m_s / 1000.0f;
  portENTER_CRITICAL(&s_us_mux);
  s_sound_speed_mm_per_us = speed_mm_per_us;
  portEXIT_CRITICAL(&s_us_mux);

  // Fast path: just copy the latest values measured by the background task.
  USBlock snapshot;
  uint8_t updated_idx = 0xFF;
  portENTER_CRITICAL(&s_us_mux);
  snapshot = s_us_cache;
  updated_idx = s_last_updated_idx;
  portEXIT_CRITICAL(&s_us_mux);
  state.us = snapshot;

  static uint32_t last_send_ms = 0;
  if (!ui::telemetry_ws_has_clients()) {
    return;
  }
  if (last_send_ms != 0 && now - last_send_ms < kTelemetryIntervalMs) {
    return;
  }
  last_send_ms = now;

  JsonDocument doc;
  doc["type"] = "us";
  doc["front_mm"] = snapshot.front_mm;
  doc["right_mm"] = snapshot.right_mm;
  doc["back_mm"] = snapshot.back_mm;
  doc["left_mm"] = snapshot.left_mm;
  doc["d45_mm"] = snapshot.d45_mm;
  if (updated_idx != 0xFF) {
    doc["updated"] = us_index_key(updated_idx);
  }
  doc["period_ms"] = kTelemetryIntervalMs;
  ui::telemetry_ws_send(doc);
}
