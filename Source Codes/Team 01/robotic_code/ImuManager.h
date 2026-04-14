#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>

class ImuManager {
private:
  uint8_t sdaPin;
  uint8_t sclPin;

  float cachedYawEulerDeg;
  float cachedYawMagDeg;   // now tilt-compensated
  float cachedPitchDeg;
  float cachedRollDeg;

  float cachedAccX;
  float cachedAccY;
  float cachedAccZ;

  uint8_t calSys, calGyro, calAccel, calMag;

  unsigned long lastUpdate;
  unsigned long updatePeriodMs;

  bool  northAnchorEnabled;
  float northAnchorOffsetDeg;

  float campusYawOffsetDeg;

  float relYawOffsetDeg;
  bool  relYawOffsetEnabled;

  float yawFiltered;
  bool  yawFilterInit;
  float yawFilterAlpha;

  static float normalizeDeg180(float a);
  static float deg2rad(float d) { return d * 0.01745329252f; }

  void  startupAlignNorthFromMag(uint16_t durationMs);
  float computeYawNorthStableDeg() const;
  float computeYawCampusDeg() const;
  float computeYawFinalDeg() const;

public:
  ImuManager();

  void IMUBegin();
  void IMUBegin(uint8_t ESP32_sda, uint8_t ESP32_scl);

  void UpdateAllData();

  bool CalibrateYaw(bool Cal_buttonSTS);

  void  SetCampusYawOffsetDeg(float offsetDeg);
  float GetCampusYawOffsetDeg() const;

  float ReadYawMag();        // tilt-compensated
  float ReadYawEuler();
  float ReadYawNorth();
  float ReadYawCampus();
  float ReadYaw();
  float ReadYawFiltered();

  void GetCalibration(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag);

  float ReadPitch();
  float ReadRoll();
  float ReadLinearAccX();
  float ReadLinearAccY();
  float ReadLinearAccZ();
};

#endif
