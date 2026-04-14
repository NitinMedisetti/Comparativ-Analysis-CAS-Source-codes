#include "ImuManager.h"

#include <Wire.h>
#include <math.h>

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

static constexpr uint8_t kBnoAddr = 0x28;
static constexpr float   kDefaultCampusYawOffsetDeg = 0.0f;
static constexpr uint16_t kStartupNorthAlignMs = 3000;

static Adafruit_BNO055 bno(55, kBnoAddr, &Wire);

float ImuManager::normalizeDeg180(float a) {
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a - 180.0f;
}

ImuManager::ImuManager() {
  sdaPin = 21;
  sclPin = 22;

  cachedYawEulerDeg = 0.0f;
  cachedYawMagDeg   = 0.0f;
  cachedPitchDeg    = 0.0f;
  cachedRollDeg     = 0.0f;

  cachedAccX = cachedAccY = cachedAccZ = 0.0f;

  calSys = calGyro = calAccel = calMag = 0;

  lastUpdate = 0;
  updatePeriodMs = 100;

  northAnchorEnabled = false;
  northAnchorOffsetDeg = 0.0f;

  campusYawOffsetDeg = kDefaultCampusYawOffsetDeg;

  relYawOffsetDeg = 0.0f;
  relYawOffsetEnabled = false;

  yawFiltered = 0.0f;
  yawFilterInit = false;
  yawFilterAlpha = 0.20f;
}

void ImuManager::IMUBegin() {
  Wire.begin(sdaPin, sclPin);

  if (!bno.begin()) {
    while (true) {
      Serial.println("IMU Init failed (Adafruit_BNO055). Retry in 5s...");
      delay(5000);
      Wire.begin(sdaPin, sclPin);
      if (bno.begin()) break;
    }
  }

  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);

  relYawOffsetEnabled = false;
  relYawOffsetDeg = 0.0f;
  yawFilterInit = false;

  for (int i = 0; i < 10; i++) {
    UpdateAllData();
    delay(20);
  }

  startupAlignNorthFromMag(kStartupNorthAlignMs);

  Serial.print("CampusYawOffsetDeg (current) = ");
  Serial.println(campusYawOffsetDeg, 2);
}

void ImuManager::IMUBegin(uint8_t ESP32_sda, uint8_t ESP32_scl) {
  sdaPin = ESP32_sda;
  sclPin = ESP32_scl;
  IMUBegin();
}

void ImuManager::SetCampusYawOffsetDeg(float offsetDeg) {
  campusYawOffsetDeg = normalizeDeg180(offsetDeg);
  yawFilterInit = false;
}

float ImuManager::GetCampusYawOffsetDeg() const {
  return campusYawOffsetDeg;
}

void ImuManager::UpdateAllData() {
  const unsigned long now = millis();
  if (now - lastUpdate < updatePeriodMs) return;
  lastUpdate = now;

  // Euler (fusion): x=Heading, y=Roll, z=Pitch
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  cachedYawEulerDeg = normalizeDeg180(euler.x());
  cachedRollDeg  = euler.y();
  cachedPitchDeg = euler.z();

  imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  cachedAccX = lin.x();
  cachedAccY = lin.y();
  cachedAccZ = lin.z();

  bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);

  // Raw mag vector
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  const float mx = mag.x();
  const float my = mag.y();
  const float mz = mag.z();

  // --- Tilt-compensated heading (minimal but correct) ---
  // Use roll/pitch to project mag vector into horizontal plane.
  const float roll  = deg2rad(cachedRollDeg);
  const float pitch = deg2rad(cachedPitchDeg);

  // Common tilt compensation (signs depend on axis conventions; this form is widely used)
  const float Xh = mx * cosf(pitch) + mz * sinf(pitch);
  const float Yh = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

  const float hNorm = sqrtf(Xh * Xh + Yh * Yh);
  if (hNorm > 1e-6f) {
    cachedYawMagDeg = normalizeDeg180(atan2f(Yh, Xh) * 180.0f / PI);
  } else {
    cachedYawMagDeg = cachedYawEulerDeg;
  }

  // Filter final yaw
  const float yawUse = computeYawFinalDeg();
  if (!yawFilterInit) {
    yawFiltered = yawUse;
    yawFilterInit = true;
  } else {
    const float delta = normalizeDeg180(yawUse - yawFiltered);
    yawFiltered = normalizeDeg180(yawFiltered + yawFilterAlpha * delta);
  }
}

void ImuManager::startupAlignNorthFromMag(uint16_t durationMs) {
  const unsigned long t0 = millis();

  float sumSinMag = 0.0f, sumCosMag = 0.0f;
  float sumSinEul = 0.0f, sumCosEul = 0.0f;
  uint16_t n = 0;

  while ((millis() - t0) < durationMs) {
    // Update caches (includes tilt-comp mag)
    // Force direct read for faster sampling:
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    const float yawEul = normalizeDeg180(euler.x());
    const float rollDeg = euler.y();
    const float pitchDeg = euler.z();

    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    const float mx = mag.x(), my = mag.y(), mz = mag.z();

    const float roll  = deg2rad(rollDeg);
    const float pitch = deg2rad(pitchDeg);

    const float Xh = mx * cosf(pitch) + mz * sinf(pitch);
    const float Yh = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

    const float hNorm = sqrtf(Xh * Xh + Yh * Yh);
    if (hNorm > 1e-6f) {
      const float yawMag = normalizeDeg180(atan2f(Yh, Xh) * 180.0f / PI);

      const float radMag = yawMag * PI / 180.0f;
      const float radEul = yawEul * PI / 180.0f;

      sumSinMag += sinf(radMag);
      sumCosMag += cosf(radMag);

      sumSinEul += sinf(radEul);
      sumCosEul += cosf(radEul);

      n++;
    }

    delay(20);
  }

  if (n < 5) {
    northAnchorEnabled = false;
    northAnchorOffsetDeg = 0.0f;
    Serial.println("WARN: Startup mag-mean failed (n<5). North anchor disabled.");
    return;
  }

  const float meanMagDeg = normalizeDeg180(atan2f(sumSinMag, sumCosMag) * 180.0f / PI);
  const float meanEulDeg = normalizeDeg180(atan2f(sumSinEul, sumCosEul) * 180.0f / PI);

  northAnchorOffsetDeg = normalizeDeg180(meanMagDeg - meanEulDeg);
  northAnchorEnabled = true;

  Serial.print("Startup Align: meanMag=");
  Serial.print(meanMagDeg, 2);
  Serial.print(" meanEuler=");
  Serial.print(meanEulDeg, 2);
  Serial.print(" anchorOffset=");
  Serial.println(northAnchorOffsetDeg, 2);
}

bool ImuManager::CalibrateYaw(bool Cal_buttonSTS) {
  if (Cal_buttonSTS) {
    relYawOffsetDeg = computeYawCampusDeg();
    relYawOffsetEnabled = true;
    yawFilterInit = false;
    return true;
  }
  return false;
}

float ImuManager::computeYawNorthStableDeg() const {
  if (!northAnchorEnabled) return cachedYawMagDeg;
  return normalizeDeg180(cachedYawEulerDeg + northAnchorOffsetDeg);
}

float ImuManager::computeYawCampusDeg() const {
  return normalizeDeg180(computeYawNorthStableDeg() - campusYawOffsetDeg);
}

float ImuManager::computeYawFinalDeg() const {
  float yaw = computeYawCampusDeg();
  if (relYawOffsetEnabled) yaw = normalizeDeg180(yaw - relYawOffsetDeg);
  return yaw;
}

float ImuManager::ReadYawMag()    { return cachedYawMagDeg; }
float ImuManager::ReadYawEuler()  { return cachedYawEulerDeg; }
float ImuManager::ReadYawNorth()  { return computeYawNorthStableDeg(); }
float ImuManager::ReadYawCampus() { return computeYawCampusDeg(); }
float ImuManager::ReadYaw()       { return computeYawFinalDeg(); }

float ImuManager::ReadYawFiltered() {
  if (!yawFilterInit) return ReadYaw();
  return yawFiltered;
}

void ImuManager::GetCalibration(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag) {
  sys = calSys; gyro = calGyro; accel = calAccel; mag = calMag;
}

float ImuManager::ReadPitch() { return cachedPitchDeg; }
float ImuManager::ReadRoll()  { return cachedRollDeg;  }

float ImuManager::ReadLinearAccX() { return cachedAccX; }
float ImuManager::ReadLinearAccY() { return cachedAccY; }
float ImuManager::ReadLinearAccZ() { return cachedAccZ; }
