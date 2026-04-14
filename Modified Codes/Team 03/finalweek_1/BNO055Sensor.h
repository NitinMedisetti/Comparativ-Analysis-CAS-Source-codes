#ifndef BNO055_SENSOR_H
#define BNO055_SENSOR_H

//#include <Arduino.h>
//#include <Wire.h>
//#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>
//#include <math.h>

// ===================== User Config =====================
#ifdef robot
  #define Heading_offset 90 // 270
  #define a1 8
  #define a2 126  
  #define a3 -42
  #define m1 89
  #define m2 3358
  #define m3 -2951
  #define g1 0
  #define g2 0
  #define g3 -1
  #define ar 1000
  #define mr 896
#else 
  #define Heading_offset 270 // 270
  #define a1 3
  #define a2 14
  #define a3 -36
  #define m1 48 
  #define m2 350
  #define m3 -646
  #define g1 0
  #define g2 -5
  #define g3 1
  #define ar 1000
  #define mr 642
#endif




#define BNO055_SAMPLERATE_DELAY_MS 100

// Apply hardcoded offsets during init
static constexpr bool IMU_USE_HARDCODED_OFFSETS = true;

// After fully calibrated, print offsets as C code to paste
static constexpr bool IMU_PRINT_OFFSETS_ON_CALIB_DONE = true;

// Try to verify offsets by reading them back
static constexpr bool IMU_VERIFY_OFFSETS_AFTER_SET = true;

// ===================== BNO055 Object =====================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ===================== Hardcoded Calibration =====================
// Replace these placeholders with your REAL calibration values.
// If you don't have them yet, run calibration once and copy the printed block.
static const adafruit_bno055_offsets_t BNO055_HARDCODED_OFFSETS = {
  /* accel_offset_x */ a1,
  /* accel_offset_y */ a2,
  /* accel_offset_z */ a3,

  /* mag_offset_x   */ m1,
  /* mag_offset_y   */ m2,
  /* mag_offset_z   */ m3,

  /* gyro_offset_x  */ g1,
  /* gyro_offset_y  */ g2,
  /* gyro_offset_z  */ g3,

  /* accel_radius   */ ar,
  /* mag_radius     */ mr
};

// ===================== State =====================
bool imuOffsetsLoaded = false;

static bool imuCalibRunning = false;
static uint32_t imuCalibStartMs = 0;
static uint32_t imuCalibTimeoutMs = 0;
static bool imuCalibReported = false;

// Global variables for orientation readings
float imuHeading = 0, imuRoll = 0, imuPitch = 0;
float imuLinAx = 0, imuLinAy = 0, imuLinAz = 0;

// ===================== Helpers =====================
static void displaySensorDetails() {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println();
  delay(200);
}

static void displaySensorStatus() {
  uint8_t system_status = 0, self_test_results = 0, system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  Serial.println();
  Serial.print("System Status: 0x"); Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x"); Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x"); Serial.println(system_error, HEX);
  Serial.println();
  delay(200);
}

static void displaySensorOffsets(const adafruit_bno055_offsets_t &c) {
  Serial.print("Accelerometer: ");
  Serial.print(c.accel_offset_x); Serial.print(" ");
  Serial.print(c.accel_offset_y); Serial.print(" ");
  Serial.print(c.accel_offset_z); Serial.println();

  Serial.print("Gyro: ");
  Serial.print(c.gyro_offset_x); Serial.print(" ");
  Serial.print(c.gyro_offset_y); Serial.print(" ");
  Serial.print(c.gyro_offset_z); Serial.println();

  Serial.print("Mag: ");
  Serial.print(c.mag_offset_x); Serial.print(" ");
  Serial.print(c.mag_offset_y); Serial.print(" ");
  Serial.print(c.mag_offset_z); Serial.println();

  Serial.print("Accel Radius: ");
  Serial.println(c.accel_radius);

  Serial.print("Mag Radius: ");
  Serial.println(c.mag_radius);
}

static void printOffsetsAsCode(const adafruit_bno055_offsets_t &c) {
  Serial.println("\n===== COPY THIS INTO BNO055_HARDCODED_OFFSETS =====");
  Serial.println("static const adafruit_bno055_offsets_t BNO055_HARDCODED_OFFSETS = {");
  Serial.printf("  /* accel_offset_x */ %d,\n", c.accel_offset_x);
  Serial.printf("  /* accel_offset_y */ %d,\n", c.accel_offset_y);
  Serial.printf("  /* accel_offset_z */ %d,\n\n", c.accel_offset_z);

  Serial.printf("  /* mag_offset_x   */ %d,\n", c.mag_offset_x);
  Serial.printf("  /* mag_offset_y   */ %d,\n", c.mag_offset_y);
  Serial.printf("  /* mag_offset_z   */ %d,\n\n", c.mag_offset_z);

  Serial.printf("  /* gyro_offset_x  */ %d,\n", c.gyro_offset_x);
  Serial.printf("  /* gyro_offset_y  */ %d,\n", c.gyro_offset_y);
  Serial.printf("  /* gyro_offset_z  */ %d,\n\n", c.gyro_offset_z);

  Serial.printf("  /* accel_radius   */ %u,\n", (unsigned)c.accel_radius);
  Serial.printf("  /* mag_radius     */ %u\n",  (unsigned)c.mag_radius);
  Serial.println("};");
  Serial.println("==================================================\n");
}

// Compare two offset structs (simple exact compare)
static bool offsetsEqual(const adafruit_bno055_offsets_t &a, const adafruit_bno055_offsets_t &b) {
  return (a.accel_offset_x == b.accel_offset_x) &&
         (a.accel_offset_y == b.accel_offset_y) &&
         (a.accel_offset_z == b.accel_offset_z) &&
         (a.mag_offset_x   == b.mag_offset_x) &&
         (a.mag_offset_y   == b.mag_offset_y) &&
         (a.mag_offset_z   == b.mag_offset_z) &&
         (a.gyro_offset_x  == b.gyro_offset_x) &&
         (a.gyro_offset_y  == b.gyro_offset_y) &&
         (a.gyro_offset_z  == b.gyro_offset_z) &&
         (a.accel_radius   == b.accel_radius) &&
         (a.mag_radius     == b.mag_radius);
}


// ===================== Calibration (Non-blocking) =====================
inline void IMU_startCalibration(uint32_t timeoutMs = 240000) {
  imuCalibRunning = true;
  imuCalibReported = false;
  imuCalibStartMs = millis();
  imuCalibTimeoutMs = timeoutMs;
  Serial.println("[IMU] Calibration started (non-blocking). Move robot in all orientations.");
}

// Call periodically. Returns true when finished (success or timeout).
inline bool IMU_calibrationStep() {
  if (!imuCalibRunning) return false;

  if (millis() - imuCalibStartMs > imuCalibTimeoutMs) {
    imuCalibRunning = false;
    Serial.println("[IMU] Calibration timeout. Continuing without full calibration.");
    return true;
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.printf("[IMU] Cal SYS:%u G:%u A:%u M:%u\n", sys, gyro, accel, mag);
  }

  if (bno.isFullyCalibrated()) {
    imuCalibRunning = false;
    Serial.println("[IMU] Fully calibrated!");

    if (!imuCalibReported) {
      adafruit_bno055_offsets_t newCalib;
      bool ok = bno.getSensorOffsets(newCalib);   // this one returns bool in most versions
      if (ok) {
        Serial.println("[IMU] Offsets read from sensor:");
        displaySensorOffsets(newCalib);

        if (IMU_PRINT_OFFSETS_ON_CALIB_DONE) {
          printOffsetsAsCode(newCalib);
        }
      } else {
        Serial.println("[IMU] Failed to read offsets from sensor (getSensorOffsets returned false).");
      }

      imuCalibReported = true;
    }
    return true;
  }

  return false;
}

inline bool IMU_isCalibrating() { return imuCalibRunning; }

// ===================== Init =====================
inline bool initIMU() {
  Serial.println("\nInitializing BNO055 IMU...");

  if (!bno.begin()) {
    Serial.println("No BNO055 detected, check wiring or address!");
    return false;
  }

  // I2C safety
  Wire.setTimeOut(50);
  Wire.setClock(100000);

  // external crystal
  bno.setExtCrystalUse(true);

  // Apply hardcoded offsets (EEPROM removed)
  if (IMU_USE_HARDCODED_OFFSETS) {
    Serial.println("[IMU] Applying HARDCODED offsets (EEPROM disabled).");

    // setSensorOffsets() is VOID in your library -> no if() check.
    bno.setSensorOffsets(BNO055_HARDCODED_OFFSETS);

    // Optional verify by reading back
    if (IMU_VERIFY_OFFSETS_AFTER_SET) {
      adafruit_bno055_offsets_t readBack;
      bool ok = bno.getSensorOffsets(readBack);
      if (ok) {
        if (offsetsEqual(readBack, BNO055_HARDCODED_OFFSETS)) {
          imuOffsetsLoaded = true;
          Serial.println("[IMU] Hardcoded offsets applied & verified.");
        } else {
          imuOffsetsLoaded = false;
          Serial.println("[IMU] Offsets set, but readback mismatch (still may work). Readback:");
          displaySensorOffsets(readBack);
        }
      } else {
        imuOffsetsLoaded = false;
        Serial.println("[IMU] Offsets set, but could not read back offsets for verification.");
      }
    } else {
      imuOffsetsLoaded = true; // assume applied
      Serial.println("[IMU] Hardcoded offsets applied (verification disabled).");
    }
  } else {
    imuOffsetsLoaded = false;
    Serial.println("[IMU] Hardcoded offsets disabled. Calibrate later if needed.");
  }

  displaySensorDetails();
  displaySensorStatus();
  Serial.println("[IMU] initIMU done (non-blocking).");
  return true;
}

// ===================== Readings =====================
inline void getIMUReadings() {
  sensors_event_t event;
  bno.getEvent(&event);

  imuHeading = event.orientation.x;
  float temp = imuHeading - Heading_offset;
  if (temp >= 360.0f) temp -= 360.0f;
  if (temp < 0.0f)     temp += 360.0f;
  imuHeading = temp;

  imuRoll  = event.orientation.y;
  imuPitch = event.orientation.z;

  imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imuLinAx = lin.x();
  imuLinAy = lin.y();
  imuLinAz = lin.z();
}

inline void returnIMUReadings(float &Heading, float &Roll, float &Pitch) {
  Heading = imuHeading;
  Roll    = imuRoll;
  Pitch   = imuPitch;
}

inline void returnIMULinearAccel(float &ax, float &ay, float &az) {
  ax = imuLinAx;
  ay = imuLinAy;
  az = imuLinAz;
}

inline double IMU_getYawRad() {
  return (double)imuHeading * M_PI / 180.0;
}

inline void IMU_getLinAccBody(float &ax, float &ay) {
  ax = imuLinAx;      // forward (+X)
  ay = -imuLinAy;     // right  (invert because +Y is left in your convention)
}

#endif // BNO055_SENSOR_H
