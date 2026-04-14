#include "sensor_imu.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <Wire.h>

#include "config/dev_params.h"
#include "timebase.h"
#include "ui/ws_telemetry.h"

// BNO055 IMU integration:
// - Auto-detects the sensor on 0x28/0x29.
// - Publishes Euler angles at `IMU_INTERVAL_MS` plus slower vector/calibration data.
// - Stores/restores calibration offsets in NVS to reduce warm-up time.

static Adafruit_BNO055 bno28(55, 0x28);
static Adafruit_BNO055 bno29(55, 0x29);
static Adafruit_BNO055* active_bno = nullptr;
static uint32_t s_last_imu_ms = 0;
static uint32_t s_last_begin_attempt_ms = 0;
static uint32_t s_last_health_check_ms = 0;
static uint32_t s_last_vectors_ms = 0;
static uint32_t s_last_cal_ms = 0;
static bool s_offsets_saved = false;
static bool s_offsets_save_attempted = false;
static uint32_t s_last_ws_send_ms = 0;

namespace {

constexpr char kPrefsNamespace[] = "imu_cal";
constexpr char kPrefsOffsetsKey[] = "offsets";
constexpr uint8_t kTempSourceAccel = 0x01;
constexpr uint8_t kSelfTestPassMask = 0x0F;
constexpr uint32_t kImuTelemetryIntervalMs = TELEMETRY_HZ ? (1000u / TELEMETRY_HZ) : 25u;
constexpr uint32_t kImuVectorsIntervalMs = 100;
constexpr uint32_t kImuCalibrationIntervalMs = 100;

Preferences s_imu_prefs;

uint8_t active_bno_address() {
  return (active_bno == &bno29) ? 0x29 : 0x28;
}

bool write_temp_source_accel() {
  Wire.beginTransmission(active_bno_address());
  Wire.write(Adafruit_BNO055::BNO055_TEMP_SOURCE_ADDR);
  Wire.write(kTempSourceAccel);
  return Wire.endTransmission() == 0;
}

bool load_calibration_offsets() {
  if (!active_bno) return false;
  if (!s_imu_prefs.begin(kPrefsNamespace, false)) {
    return false;
  }
  bool loaded = false;
  if (s_imu_prefs.isKey(kPrefsOffsetsKey)) {
    adafruit_bno055_offsets_t offsets;
    const size_t len = s_imu_prefs.getBytes(kPrefsOffsetsKey, &offsets, sizeof(offsets));
    if (len == sizeof(offsets)) {
      active_bno->setSensorOffsets(offsets);
      loaded = true;
    }
  }
  s_imu_prefs.end();
  return loaded;
}

bool save_calibration_offsets() {
  if (!active_bno) return false;
  adafruit_bno055_offsets_t offsets;
  if (!active_bno->getSensorOffsets(offsets)) {
    return false;
  }
  if (!s_imu_prefs.begin(kPrefsNamespace, false)) {
    return false;
  }
  const size_t written = s_imu_prefs.putBytes(kPrefsOffsetsKey, &offsets, sizeof(offsets));
  s_imu_prefs.end();
  return written == sizeof(offsets);
}

bool run_post_and_status_checks() {
  if (!active_bno) return false;
  uint8_t system_status = 0;
  uint8_t self_test = 0;
  uint8_t system_error = 0;
  active_bno->getSystemStatus(&system_status, &self_test, &system_error);

  if (system_error) {
    Serial.printf("BNO055 system error %u (status=0x%02X)\n", system_error, system_status);
    return false;
  }
  if (system_status == 0x01) {
    Serial.println("BNO055 reported system error state.");
    return false;
  }
  if ((self_test & kSelfTestPassMask) != kSelfTestPassMask) {
    // Some BNO055 modules report incomplete self-test bits despite functioning sensor
    // readouts. Treat this as a warning instead of a hard failure so IMU data still
    // flows to the UI/OLED.
    Serial.printf("BNO055 self-test incomplete (0x%02X)\n", self_test);
  }
  return true;
}

void clear_state(AppState& state) {
  state.imu.stamp = {};
  state.imu.yaw = state.imu.pitch = state.imu.roll = state.imu.tempC = NAN;
  state.imu.linear_accel_x = state.imu.linear_accel_y = state.imu.linear_accel_z = NAN;
  state.imu.accel_x = state.imu.accel_y = state.imu.accel_z = NAN;
  state.imu.gyro_x = state.imu.gyro_y = state.imu.gyro_z = NAN;
  state.imu.mag_x = state.imu.mag_y = state.imu.mag_z = NAN;
  state.imu.cal_sys = state.imu.cal_gyro = state.imu.cal_accel = state.imu.cal_mag = 0;
  state.imu.calibrated = false;
  state.imu.valid = false;
}

}  // namespace

bool sensor_imu_begin() {
  bool has28 = false, has29 = false;
  Wire.beginTransmission(0x28);
  has28 = (Wire.endTransmission() == 0);
  Wire.beginTransmission(0x29);
  has29 = (Wire.endTransmission() == 0);
  if (has28) {
    active_bno = &bno28;
  } else if (has29) {
    active_bno = &bno29;
  } else {
    active_bno = nullptr;
    Serial.println("BNO055 not detected on I2C (0x28/0x29).");
    return false;
  }

  s_offsets_saved = false;
  s_offsets_save_attempted = false;
  s_last_vectors_ms = 0;
  s_last_cal_ms = 0;

  Serial.printf("BNO055 detected on 0x%02X\n", active_bno_address());

  for (int i = 0; i < 5; i++) {
    if (active_bno->begin(OPERATION_MODE_CONFIG)) {
      delay(50);
      active_bno->setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
      active_bno->setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);
      write_temp_source_accel();
      load_calibration_offsets();  // reuse stored calibration per datasheet 3.10.4
      // Not all BNO055 breakout boards have a stable external 32.768kHz crystal wired in.
      // For portability, prefer the internal oscillator; this avoids "all zeros" readouts
      // observed on some modules when ext crystal is forced on.
      active_bno->setExtCrystalUse(false);
      active_bno->setMode(OPERATION_MODE_NDOF);
      delay(120);
      if (!run_post_and_status_checks()) {
        active_bno = nullptr;
        return false;
      }
      return true;
    }
    Serial.println("BNO055 begin() failed, retrying...");
    delay(200);
  }
  Serial.println("BNO055 begin() failed (giving up).");
  active_bno = nullptr;
  return false;
}

void sensor_imu_update(AppState& state) {
  const uint32_t now = millis();
  if (!active_bno) {
    clear_state(state);
    if ((uint32_t)(now - s_last_begin_attempt_ms) < 2000) {
      return;
    }
    s_last_begin_attempt_ms = now;
    if (sensor_imu_begin()) {
      Serial.println("BNO055 re-init OK");
    } else {
      Serial.println("BNO055 re-init failed");
    }
    return;
  }

  if ((now - s_last_imu_ms) < IMU_INTERVAL_MS) return;
  s_last_imu_ms = now;

  if ((uint32_t)(now - s_last_health_check_ms) >= 3000) {
    s_last_health_check_ms = now;
    if (!run_post_and_status_checks()) {
      Serial.println("BNO055 health-check failed; restarting...");
      sensor_imu_restart(state);
      return;
    }
  }

  state.imu.stamp.t_us = now_us();
  state.imu.stamp.seq += 1;

  sensors_event_t orientation;
  active_bno->getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
  state.imu.yaw = orientation.orientation.x;
  state.imu.pitch = orientation.orientation.y;
  state.imu.roll = orientation.orientation.z;

  const bool do_vectors =
      !s_last_vectors_ms || (uint32_t)(now - s_last_vectors_ms) >= kImuVectorsIntervalMs;
  if (do_vectors) {
    s_last_vectors_ms = now;

    sensors_event_t linear_accel;
    active_bno->getEvent(&linear_accel, Adafruit_BNO055::VECTOR_LINEARACCEL);
    state.imu.linear_accel_x = linear_accel.acceleration.x;
    state.imu.linear_accel_y = linear_accel.acceleration.y;
    state.imu.linear_accel_z = linear_accel.acceleration.z;

    sensors_event_t accel;
    active_bno->getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    state.imu.accel_x = accel.acceleration.x;
    state.imu.accel_y = accel.acceleration.y;
    state.imu.accel_z = accel.acceleration.z;

    sensors_event_t gyro;
    active_bno->getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
    state.imu.gyro_x = gyro.gyro.x;
    state.imu.gyro_y = gyro.gyro.y;
    state.imu.gyro_z = gyro.gyro.z;

    sensors_event_t mag;
    active_bno->getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    state.imu.mag_x = mag.magnetic.x;
    state.imu.mag_y = mag.magnetic.y;
    state.imu.mag_z = mag.magnetic.z;

    state.imu.tempC = active_bno->getTemp();
  }

  const bool do_cal =
      !s_last_cal_ms || (uint32_t)(now - s_last_cal_ms) >= kImuCalibrationIntervalMs;
  if (do_cal) {
    s_last_cal_ms = now;

    uint8_t cal_sys = 0, cal_gyro = 0, cal_accel = 0, cal_mag = 0;
    active_bno->getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);
    state.imu.cal_sys = cal_sys;
    state.imu.cal_gyro = cal_gyro;
    state.imu.cal_accel = cal_accel;
    state.imu.cal_mag = cal_mag;

    const bool fully_calibrated =
        (cal_sys == 3 && cal_gyro == 3 && cal_accel == 3 && cal_mag == 3);
    if (fully_calibrated && !s_offsets_saved && !s_offsets_save_attempted) {
      s_offsets_save_attempted = true;
      if (save_calibration_offsets()) {
        s_offsets_saved = true;
        Serial.println("BNO055 calibration profile saved to NVS");
      } else {
        Serial.println("BNO055 calibration save failed");
      }
    } else if (!fully_calibrated) {
      s_offsets_save_attempted = false;
    }
  }

  // Only report calibrated once all subsystems are done or a stored profile is active.
  const bool fully_calibrated =
      (state.imu.cal_sys == 3 && state.imu.cal_gyro == 3 && state.imu.cal_accel == 3 &&
       state.imu.cal_mag == 3);
  state.imu.calibrated = fully_calibrated || s_offsets_saved;
  state.imu.valid = true;

  if (!ui::telemetry_ws_has_clients()) {
    return;
  }
  if (s_last_ws_send_ms != 0 && (uint32_t)(now - s_last_ws_send_ms) < kImuTelemetryIntervalMs) {
    return;
  }
  s_last_ws_send_ms = now;

  JsonDocument doc;
  doc["type"] = "imu";
  doc["yaw"] = state.imu.yaw;
  doc["cal_sys"] = state.imu.cal_sys;
  doc["cal_gyro"] = state.imu.cal_gyro;
  doc["cal_accel"] = state.imu.cal_accel;
  doc["cal_mag"] = state.imu.cal_mag;
  ui::telemetry_ws_send(doc);
}

bool sensor_imu_restart(AppState& state) {
  active_bno = nullptr;
  s_last_imu_ms = 0;
  s_last_vectors_ms = 0;
  s_last_cal_ms = 0;
  s_offsets_saved = false;
  s_offsets_save_attempted = false;
  clear_state(state);
  return sensor_imu_begin();
}
