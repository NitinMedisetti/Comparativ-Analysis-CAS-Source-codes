/* ================================================================
   TEAM4 — ZONE 1 + ZONE 2 (NEW PATH) + ZONE 3 + ROBO CONTROL UI (SSE) + OLED

   ✅ GLOBAL FEATURES (reusable for ALL zones / states)
   1) GLOBAL IMU TURN SERVICE (closed-loop + overshoot correction)
   2) GLOBAL STEP LEFT/RIGHT (side-step override)
   3) ✅ NEW: GLOBAL HEADING HOLD for FORWARD (yaw kept within ±5°)

   ================================================================ */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

// ✅ REQUIRED for AsyncWebServer + SSE
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>

#include "Controller.h"   // ✅ uses initialcheckuproutine() from here
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
// After all the includes, before AP_SSID
static bool serverStarted = false;

// ---------------- AP CONFIG ----------------
static const char* AP_SSID = "SWAS2026";
static const char* AP_PASS = "12345678";

AsyncWebServer server(80);
AsyncEventSource events("/events");

// ---------------- IMU + OLED (I2C) ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

static const int OLED_W = 128;
static const int OLED_H = 32;
static const int OLED_RESET = -1;
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);

static const uint8_t OLED_ADDR = 0x3C;

// I2C mutex (IMU + OLED share bus)
SemaphoreHandle_t i2cMutex;

// ---------------- GPS (kept; WP updated) ----------------
static const int GPS_RX_PIN = 16;     // ESP32 RX  <- GPS TX
static const int GPS_TX_PIN = 17;     // ESP32 TX  -> GPS RX
static const uint32_t GPS_BAUD = 9600;

HardwareSerial GPSSerial(2);
TinyGPSPlus gps;

portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED;
volatile double g_lat = 0.0;
volatile double g_lon = 0.0;
volatile bool   g_fix = false;
volatile uint32_t g_lastFixMs = 0;

// Waypoints
static const double Z2_WP1_LAT = 48.8300000;
static const double Z2_WP1_LON = 12.9550000;

static const double Z2_WP2_LAT = 48.8299070;
static const double Z2_WP2_LON = 12.9549200;

// Waypoint zone radius requirement
static const float  WP_ENTER_M = 5.0f;
static const float  WP_EXIT_M  = 6.5f;
static const int    WP_CONFIRM_N = 5;

volatile bool wp1Inside=false, wp1Passed=false;
volatile bool wp2Inside=false, wp2Passed=false;
volatile int  wp1InCnt=0, wp1OutCnt=0;
volatile int  wp2InCnt=0, wp2OutCnt=0;

// ---------------- Ultrasonic helper ----------------
class UltrasonicHC {
public:
  UltrasonicHC(int trigPin, int echoPin) : trig(trigPin), echo(echoPin) {}
  void begin() {
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(trig, LOW);
  }
  float readCM(uint16_t maxCm) {
    const uint32_t timeout_us = (uint32_t)maxCm * 58UL + 4000UL;

    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    uint32_t dur = pulseIn(echo, HIGH, timeout_us);
    if (dur == 0) return (float)maxCm + 1.0f;

    float cm = (float)dur / 58.0f;
    if (cm < 0) cm = 0;
    if (cm > (float)maxCm + 1.0f) cm = (float)maxCm + 1.0f;
    return cm;
  }
private:
  int trig, echo;
};

// Pins (your wiring)
UltrasonicHC usFront(32, 25);
UltrasonicHC usBack (5,  14);
UltrasonicHC usLeft (4,  18);
UltrasonicHC usRight(13, 19);

// ---------------- Parameters ----------------
static const uint16_t US_MAX_CM = 250;
static const float FILTER_ALPHA = 0.35f;

// “Default” wall target
static const float WALL_SETPOINT_CM = 100.0f;
static const float WALL_BAND_CM     = 10.0f;

// Zone 3 specific setpoints
static const float Z3_RIGHT_ALIGN_CM = 50.0f;
static const float Z3_LEFT_ALIGN_CM  = 200.0f;

// FRONT stop target (kept)
static const float FRONT_SETPOINT_CM = 70.0f;
static const float FRONT_BAND_CM     = 10.0f;

// Clear definition
static const float SIDE_CLEAR_THRESHOLD_CM = 220.0f;
static const int   SIDE_CLEAR_CONFIRM_N    = 5;
static const int   SIDE_FOUND_CONFIRM_N    = 4;

// Alignment confirm
static const int   WALL_ALIGN_CONFIRM_N    = 8;

// Front reach confirm
static const int   FRONT_REACH_CONFIRM_N   = 5;

// Drive feel (placeholder)
static const int   BASE_SPEED = 160;
static const float KP_WALL    = 1.4f;

// GLOBAL stepping
static const bool  AUTO_STEP_ENABLED = true;
static const float STEP_ERROR_CM = 25.0f;
static const uint32_t STEP_DURATION_MS = 280;

// NEW CORNER + RIGHT-WALL targets
static const float Z12_CORNER_FRONT_CM = 50.0f;
static const float Z12_CORNER_FRONT_BAND = 10.0f;
static const float Z12_CORNER_RIGHT_CM = 30.0f;
static const float Z12_CORNER_RIGHT_BAND = 10.0f;

static const float Z12_RIGHT_FOLLOW1_CM = 70.0f;  // first corridor
static const float Z12_RIGHT_FOLLOW2_CM = 50.0f;  // second corridor

// Update rates
static const uint32_t SSE_PERIOD_MS     = 80;
static const uint32_t CONTROL_PERIOD_MS = 50;
static const uint32_t IMU_PERIOD_MS     = 20;
static const uint32_t US_PERIOD_MS      = 35;
static const uint32_t OLED_PERIOD_MS    = 120;

// ---------------- Telemetry ----------------
struct Telemetry {
  float f=251, b=251, l=251, r=251;
  float yaw=0;

  bool autoEnabled=false;

  char state[24]  = "IDLE";
  char action[24] = "STOP";

  float turnRemaining = 0;

  int c1=0, c2=0, c3=0, c4=0;

  uint8_t wpPassed = 0;  // 0..2
};

Telemetry T;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- State machine ----------------
enum AutoState : uint8_t {
  IDLE = 0,

  // NEW Zone 1+2 route
  Z12_FORWARD_TO_CORNER,
  Z12_ALIGN_RIGHT_70,
  Z12_FOLLOW_RIGHT_70,
  Z12_FORWARD_TO_RIGHT_WALL_2,
  Z12_ALIGN_RIGHT_50,
  Z12_FOLLOW_RIGHT_50,

  // Zone3 (unchanged)
  Z3_FORWARD_TO_FRONT,
  Z3_ALIGN_LEFT_100,
  Z3_FOLLOW_LEFT_WALL,
  Z3_FORWARD_TO_FRONT_2,
  Z3_ALIGN_LEFT_200,
  Z3_DONE,

  TURNING
};
volatile AutoState state = IDLE;

// ================================================================
// Controller.h integration plumbing
// ================================================================
static SemaphoreHandle_t motorMutex = nullptr;

static volatile bool initRoutineRunning = false;
static volatile bool checkupRequested = false;
static volatile bool checkupRunning   = false;

// ✅ Use Controller.h SPEED_LIMIT directly (28)
#define DRIVE_PERCENT      SPEED_LIMIT
#define TURN_PERCENT       SPEED_LIMIT
#define BODY_STEP_PERCENT  SPEED_LIMIT

// ================================================================
// Utils
// ================================================================
static float wrap360(float a) {
  while (a < 0) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}
static float angleDiff(float target, float current) {
  float diff = fmodf((target - current + 540.0f), 360.0f) - 180.0f;
  return diff;
}
static float ema(float prev, float x) {
  return FILTER_ALPHA * x + (1.0f - FILTER_ALPHA) * prev;
}
static bool sideIsClear(float sideCm) {
  return (sideCm >= SIDE_CLEAR_THRESHOLD_CM) || (sideCm > (float)US_MAX_CM);
}
static bool inBand(float cm, float target, float band) {
  return (cm >= (target - band)) && (cm <= (target + band));
}
static bool frontReached(float frontCm, float target, float band) {
  return (frontCm > 0.0f) && (frontCm <= (target + band));
}

// GPS helpers
static double deg2rad(double d){ return d * (M_PI / 180.0); }
static float haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat/2.0)*sin(dLat/2.0) +
             cos(deg2rad(lat1))*cos(deg2rad(lat2)) *
             sin(dLon/2.0)*sin(dLon/2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return (float)(R * c);
}
static void gpsSnapshot(double &lat, double &lon, bool &fix, uint32_t &ageMs) {
  portENTER_CRITICAL(&gpsMux);
  lat = g_lat; lon = g_lon; fix = g_fix;
  uint32_t lf = g_lastFixMs;
  portEXIT_CRITICAL(&gpsMux);
  ageMs = fix ? (millis() - lf) : 0xFFFFFFFFu;
}
static float gpsDistTo(double wLat, double wLon) {
  double lat, lon; bool fix; uint32_t age;
  gpsSnapshot(lat, lon, fix, age);
  if (!fix || age > 1500) return 1e9f;
  return haversineMeters(lat, lon, wLat, wLon);
}

// ---------------- MOTOR / GAIT CONTROL ----------------
static void setAction(const char* a) {
  portENTER_CRITICAL(&mux);
  strncpy(T.action, a, sizeof(T.action) - 1);
  T.action[sizeof(T.action) - 1] = '\0';
  portEXIT_CRITICAL(&mux);
}

static void motorsStop() {
  setAction("STOP");
  if (checkupRunning) return;
  if (!motorMutex) { 
    // stop();
    displayMovement("stop()");
    return; 
  }
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    // stop();
    displayMovement("stop()");
    xSemaphoreGive(motorMutex);
  }
}

static void driveStraight(int pwm) {
  (void)pwm;
  setAction("FORWARD");
  if (checkupRunning) return;
  if (!motorMutex) { 
    // forward(DRIVE_PERCENT);
    displayMovement("forward(28)");
    return; 
  }
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    // forward(DRIVE_PERCENT);
    displayMovement("forward(28)");
    xSemaphoreGive(motorMutex);
  }
}

static void driveBackward(int pwm) {
  (void)pwm;
  setAction("BACKWARD");
  if (checkupRunning) return;
  if (!motorMutex) { 
    // backward(DRIVE_PERCENT);
    displayMovement("backward(28)");
    return; 
  }
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    // backward(DRIVE_PERCENT);
    displayMovement("backward(28)");
    xSemaphoreGive(motorMutex);
  }
}

static void driveDifferential(int leftPWM, int rightPWM) {
  (void)leftPWM; (void)rightPWM;
  setAction("FORWARD");
  if (checkupRunning) return;
  if (!motorMutex) { 
    // forward(DRIVE_PERCENT);
    displayMovement("forward(28)");
    return; 
  }
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    // forward(DRIVE_PERCENT);
    displayMovement("forward(28)");
    xSemaphoreGive(motorMutex);
  }
}

static void turnInPlaceSigned(int pwmSigned) {
  setAction((pwmSigned < 0) ? "TURN_LEFT" : "TURN_RIGHT");
  if (checkupRunning) return;

  if (pwmSigned == 0) { 
    motorsStop(); 
    return; 
  }

   if (!motorMutex) {
    if (pwmSigned < 0) { 
      // rotateLeft(SPEED_LIMIT);
      displayMovement("rotateLeft(28)");
    } else { 
      // rotateRight(SPEED_LIMIT);
      displayMovement("rotateRight(28)");
    }
    return;
  }
 if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (pwmSigned < 0) { 
      // rotateLeft(SPEED_LIMIT);
      displayMovement("rotateLeft(28)");
    } else { 
      // rotateRight(SPEED_LIMIT);
      displayMovement("rotateRight(28)");
    }
    xSemaphoreGive(motorMutex);
  }
}

static void stepLeft()  {
  setAction("STEP_LEFT");
  if (checkupRunning) return;
  if (!motorMutex) { 
    // stepLeft(BODY_STEP_PERCENT);
    displayMovement("stepLeft(15)");
    return; 
  }
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    // stepLeft(BODY_STEP_PERCENT);
    displayMovement("stepLeft(15)");
    xSemaphoreGive(motorMutex);
  }
}
static void stepRight() {
  setAction("STEP_RIGHT");
  if (checkupRunning) return;
  if (!motorMutex) { 
    // stepRight(BODY_STEP_PERCENT);
    displayMovement("stepRight(15)");
    return; 
  }
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    // stepRight(BODY_STEP_PERCENT);
    displayMovement("stepRight(15)");
    xSemaphoreGive(motorMutex);
  }
}

static void setStateName(const char* s) {
  portENTER_CRITICAL(&mux);
  strncpy(T.state, s, sizeof(T.state) - 1);
  T.state[sizeof(T.state) - 1] = '\0';
  portEXIT_CRITICAL(&mux);
}

// ================================================================
// GLOBAL STEP OVERRIDE
// ================================================================
volatile bool stepActive = false;
volatile bool stepIssued = false;
volatile int  stepDir = 0;            // -1 left, +1 right
volatile uint32_t stepEndMs = 0;

// ================================================================
// ✅ GLOBAL IMU TURN SERVICE
// ================================================================
static const float TURN_TOL_DEG        = 3.0f;
static const int   TURN_STABLE_N       = 6;

static const int   TURN_PWM_MAX        = 200;
static const int   TURN_PWM_MIN_COARSE = 90;
static const int   TURN_PWM_MIN_FINE   = 45;
static const float TURN_FINE_ZONE_DEG  = 12.0f;

static const float TURN_KP_PWM_PER_DEG   = 2.2f;
static const float TURN_KD_PWM_PER_DEGPS = 0.02f;

volatile bool   turnActive      = false;
volatile float  turnTargetYaw   = 0.0f;
volatile int    turnStableCount = 0;
volatile float  turnPrevErrDeg  = 0.0f;
volatile uint32_t turnLastMs    = 0;

volatile AutoState turnNextState = IDLE;
char turnNextName[24] = "IDLE";

// ================================================================
// ✅ NEW: HEADING HOLD (FOR FORWARD ONLY)
// ================================================================
static const bool  HEADING_HOLD_ENABLED   = true;
static const float HEADING_HOLD_BAND_DEG  = 5.0f;   // your requirement: ±5°
static const float HEADING_HOLD_KP_PWM    = 8.0f;   // deg -> pwm mapping
static const int   HEADING_PWM_MIN        = 70;
static const int   HEADING_PWM_MAX        = 160;

volatile float forwardYawRef = 0.0f;      // “yaw=0” for current forward segment
volatile bool  headingCorrActive = false;
volatile float headingCorrRemaining = 0.0f;
volatile int   headingCorrDir = 0;        // -1 left, +1 right

static void resetForwardHeading(float yawNow) {
  forwardYawRef = yawNow;
  headingCorrActive = false;
  headingCorrRemaining = 0.0f;
  headingCorrDir = 0;
}

static void forwardWithHeadingHold(float yawNow, int pwmForward) {
  if (!HEADING_HOLD_ENABLED) {
    driveStraight(pwmForward);
    headingCorrActive = false;
    headingCorrRemaining = 0.0f;
    headingCorrDir = 0;
    return;
  }

  // If step or big turn service is active, don't fight it.
  if (stepActive || (state == TURNING && turnActive)) {
    headingCorrActive = false;
    headingCorrRemaining = 0.0f;
    headingCorrDir = 0;
    return;
  }

  float diff = angleDiff(forwardYawRef, yawNow); // target - current (signed)
  float rem  = fabsf(diff);

  if (rem <= HEADING_HOLD_BAND_DEG) {
    headingCorrActive = false;
    headingCorrRemaining = 0.0f;
    headingCorrDir = 0;
    driveStraight(pwmForward);
    return;
  }

  // Correct back to reference
  headingCorrActive = true;
  headingCorrRemaining = rem;
  headingCorrDir = (diff < 0) ? -1 : +1;

  int pwm = (int)lroundf(rem * HEADING_HOLD_KP_PWM);
  pwm = constrain(pwm, HEADING_PWM_MIN, HEADING_PWM_MAX);

  // Turn in place a bit until we're back within ±5°
  turnInPlaceSigned((headingCorrDir < 0) ? -pwm : +pwm);
}

static void requestTurn(float yawNow, float signedDeg,
                        AutoState nextState, const char* nextName)
{
  turnTargetYaw   = wrap360(yawNow + signedDeg);
  turnActive      = true;
  turnStableCount = 0;
  turnPrevErrDeg  = 0.0f;
  turnLastMs      = 0;

  // stop heading correction during main turns
  headingCorrActive = false;
  headingCorrRemaining = 0.0f;
  headingCorrDir = 0;

  stepActive = false;
  stepIssued = false;
  stepDir = 0;

  turnNextState = nextState;
  strncpy(turnNextName, nextName, sizeof(turnNextName)-1);
  turnNextName[sizeof(turnNextName)-1] = '\0';

  portENTER_CRITICAL(&mux);
  T.turnRemaining = fabsf(signedDeg);
  portEXIT_CRITICAL(&mux);

  setStateName((signedDeg < 0) ? "TURN_LEFT" : "TURN_RIGHT");
  state = TURNING;
}

static bool updateTurn(float yawNow)
{
  float err = angleDiff(turnTargetYaw, yawNow);
  float remaining = fabsf(err);

  portENTER_CRITICAL(&mux);
  T.turnRemaining = remaining;
  portEXIT_CRITICAL(&mux);

  if (remaining <= TURN_TOL_DEG) {
    turnStableCount++;
    motorsStop();
    if (turnStableCount >= TURN_STABLE_N) {
      portENTER_CRITICAL(&mux);
      T.turnRemaining = 0.0f;
      portEXIT_CRITICAL(&mux);
      return true;
    }
    return false;
  }

  turnStableCount = 0;

  uint32_t nowMs = millis();
  float dt = (turnLastMs == 0) ? (CONTROL_PERIOD_MS / 1000.0f)
                               : ((nowMs - turnLastMs) / 1000.0f);
  if (dt < 0.005f) dt = (CONTROL_PERIOD_MS / 1000.0f);
  turnLastMs = nowMs;

  float dErr = (err - turnPrevErrDeg) / dt;
  turnPrevErrDeg = err;

  float u = (remaining > TURN_FINE_ZONE_DEG)
              ? (TURN_KP_PWM_PER_DEG * err)
              : (TURN_KP_PWM_PER_DEG * err + TURN_KD_PWM_PER_DEGPS * dErr);

  int pwm = (int)lroundf(fabsf(u));
  int pwmMin = (remaining <= TURN_FINE_ZONE_DEG) ? TURN_PWM_MIN_FINE : TURN_PWM_MIN_COARSE;
  pwm = constrain(pwm, pwmMin, TURN_PWM_MAX);

  int signedPwm = (u < 0) ? -pwm : +pwm;
  turnInPlaceSigned(signedPwm);
  return false;
}

// ================================================================
// WALL FOLLOW HELPERS
// ================================================================
static void wallFollowLeft_Default100(float leftCm) {
  float err = (leftCm - WALL_SETPOINT_CM);

  if (AUTO_STEP_ENABLED && fabsf(err) > STEP_ERROR_CM && !stepActive) {
    stepActive = true;
    stepIssued = false;
    stepDir = (err > 0) ? -1 : +1;
    stepEndMs = millis() + STEP_DURATION_MS;
    setAction(stepDir < 0 ? "STEP_LEFT" : "STEP_RIGHT");
    return;
  }

  float corr = KP_WALL * err;
  int leftPWM  = (int)lroundf((float)BASE_SPEED - corr);
  int rightPWM = (int)lroundf((float)BASE_SPEED + corr);

  leftPWM  = constrain(leftPWM,  0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  driveDifferential(leftPWM, rightPWM);
}

static void wallFollowRight_Target(float rightCm, float targetCm) {
  float err = (rightCm - targetCm);

  if (AUTO_STEP_ENABLED && fabsf(err) > STEP_ERROR_CM && !stepActive) {
    stepActive = true;
    stepIssued = false;
    stepDir = (err > 0) ? +1 : -1;
    stepEndMs = millis() + STEP_DURATION_MS;
    setAction(stepDir < 0 ? "STEP_LEFT" : "STEP_RIGHT");
    return;
  }

  float corr = KP_WALL * err;
  int leftPWM  = (int)lroundf((float)BASE_SPEED + corr);
  int rightPWM = (int)lroundf((float)BASE_SPEED - corr);

  leftPWM  = constrain(leftPWM,  0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  driveDifferential(leftPWM, rightPWM);
}

static void wallFollowLeft_Z3_200(float leftCm) {
  float err = (leftCm - Z3_LEFT_ALIGN_CM);

  if (AUTO_STEP_ENABLED && fabsf(err) > STEP_ERROR_CM && !stepActive) {
    stepActive = true;
    stepIssued = false;
    stepDir = (err > 0) ? -1 : +1;
    stepEndMs = millis() + STEP_DURATION_MS;
    setAction(stepDir < 0 ? "STEP_LEFT" : "STEP_RIGHT");
    return;
  }

  float corr = KP_WALL * err;
  int leftPWM  = (int)lroundf((float)BASE_SPEED - corr);
  int rightPWM = (int)lroundf((float)BASE_SPEED + corr);

  leftPWM  = constrain(leftPWM,  0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  driveDifferential(leftPWM, rightPWM);
}

// ================================================================
// OLED
// ================================================================

// Helper function to display movement function calls on OLED
static void displayMovement(const char* functionName) {
  if (!i2cMutex) return;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(15)) != pdTRUE) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Movement Call:");
  display.setCursor(0, 16);
  display.print(functionName);
  display.display();
  
  xSemaphoreGive(i2cMutex);
}

static void updateOLED(const Telemetry& snap) {
  if (!i2cMutex) return;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(15)) != pdTRUE) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(snap.state);

  display.setCursor(0, 16);

  if (!snap.autoEnabled) {
    display.print("STOP");
  } else {
    if (state == TURNING && turnActive) {
      float diff = angleDiff(turnTargetYaw, snap.yaw);
      int rem = (int)lroundf(fabsf(diff));
      if (rem <= (int)lroundf(TURN_TOL_DEG)) display.print("HOLD");
      else {
        display.print((diff < 0) ? "TL " : "TR ");
        display.print(rem);
        display.print("deg");
      }
    } else if (headingCorrActive) {
      int rem = (int)lroundf(headingCorrRemaining);
      display.print((headingCorrDir < 0) ? "TL " : "TR ");
      display.print(rem);
      display.print("deg");
    } else if (stepActive) {
      display.print(stepDir < 0 ? "STEP LEFT" : "STEP RIGHT");
    } else {
      display.print(snap.action);
    }
  }

  display.display();
  xSemaphoreGive(i2cMutex);
}

// ================================================================
// Guidance (web)
// ================================================================
static String makeGuidance(const Telemetry& snap) {
  if (!snap.autoEnabled) return "STOP";
  if (state == Z3_DONE) return "Z3 DONE - ARRIVED";

  if (stepActive) return (stepDir < 0) ? "STEP LEFT" : "STEP RIGHT";

  if (state == TURNING && turnActive) {
    float diff = angleDiff(turnTargetYaw, snap.yaw);
    int rem = (int)lroundf(fabsf(diff));
    if (rem <= (int)lroundf(TURN_TOL_DEG)) return "HOLD";
    return (diff < 0) ? ("TURN LEFT " + String(rem) + " DEG")
                      : ("TURN RIGHT " + String(rem) + " DEG");
  }

  if (headingCorrActive) {
    int rem = (int)lroundf(headingCorrRemaining);
    return (headingCorrDir < 0) ? ("TURN LEFT " + String(rem) + " DEG")
                                : ("TURN RIGHT " + String(rem) + " DEG");
  }

  return String(snap.action);
}

// ================================================================
// Tasks
// ================================================================
void gpsTask(void* pv) {
  for (;;) {
    while (GPSSerial.available()) gps.encode((char)GPSSerial.read());

    if (gps.location.isUpdated() && gps.location.isValid()) {
      portENTER_CRITICAL(&gpsMux);
      g_lat = gps.location.lat();
      g_lon = gps.location.lng();
      g_fix = true;
      g_lastFixMs = millis();
      portEXIT_CRITICAL(&gpsMux);
    }

    double lat, lon; bool fix; uint32_t age;
    gpsSnapshot(lat, lon, fix, age);
    if (!fix || age > 1500) {
      vTaskDelay(pdMS_TO_TICKS(40));
      continue;
    }

    float d1 = gpsDistTo(Z2_WP1_LAT, Z2_WP1_LON);
    if (!wp1Passed) {
      if (!wp1Inside) {
        if (d1 <= WP_ENTER_M) wp1InCnt++; else wp1InCnt = 0;
        if (wp1InCnt >= WP_CONFIRM_N) { wp1Inside = true; wp1InCnt = 0; wp1OutCnt = 0; }
      } else {
        if (d1 >= WP_EXIT_M) wp1OutCnt++; else wp1OutCnt = 0;
        if (wp1OutCnt >= WP_CONFIRM_N) { wp1Inside = false; wp1Passed = true; wp1OutCnt = 0; }
      }
    }

    if (wp1Passed && !wp2Passed) {
      float d2 = gpsDistTo(Z2_WP2_LAT, Z2_WP2_LON);
      if (!wp2Inside) {
        if (d2 <= WP_ENTER_M) wp2InCnt++; else wp2InCnt = 0;
        if (wp2InCnt >= WP_CONFIRM_N) { wp2Inside = true; wp2InCnt = 0; wp2OutCnt = 0; }
      } else {
        if (d2 >= WP_EXIT_M) wp2OutCnt++; else wp2OutCnt = 0;
        if (wp2OutCnt >= WP_CONFIRM_N) { wp2Inside = false; wp2Passed = true; wp2OutCnt = 0; }
      }
    }

    uint8_t passed = (wp1Passed ? 1 : 0) + (wp2Passed ? 1 : 0);
    portENTER_CRITICAL(&mux);
    T.wpPassed = passed;
    portEXIT_CRITICAL(&mux);

    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

void sensorTask(void* pv) {
  uint32_t lastIMU = 0;
  uint32_t lastUS  = 0;
  uint8_t usIdx = 0;

  for (;;) {
    uint32_t now = millis();

    if (now - lastIMU >= IMU_PERIOD_MS) {
      lastIMU = now;
      if (i2cMutex && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
        sensors_event_t event;
        bno.getEvent(&event);
        float yaw = wrap360(event.orientation.x);
        xSemaphoreGive(i2cMutex);

        portENTER_CRITICAL(&mux);
        T.yaw = yaw;
        portEXIT_CRITICAL(&mux);
      }
    }

    if (now - lastUS >= US_PERIOD_MS) {
      lastUS = now;

      float cm = 251;
      if (usIdx == 0) cm = usFront.readCM(US_MAX_CM);
      if (usIdx == 1) cm = usBack .readCM(US_MAX_CM);
      if (usIdx == 2) cm = usLeft .readCM(US_MAX_CM);
      if (usIdx == 3) cm = usRight.readCM(US_MAX_CM);

      portENTER_CRITICAL(&mux);
      if (usIdx == 0) T.f = ema(T.f, cm);
      if (usIdx == 1) T.b = ema(T.b, cm);
      if (usIdx == 2) T.l = ema(T.l, cm);
      if (usIdx == 3) T.r = ema(T.r, cm);
      portEXIT_CRITICAL(&mux);

      usIdx = (usIdx + 1) & 0x03;
    }

    vTaskDelay(1);
  }
}

void controlTask(void* pv) {
  setStateName("IDLE");
  motorsStop();

  uint32_t lastOLED = 0;

  for (;;) {
    uint32_t now = millis();

    if (stepActive && (int32_t)(now - stepEndMs) >= 0) {
      stepActive = false;
      stepIssued = false;
      stepDir = 0;
    }

    Telemetry snap;
    portENTER_CRITICAL(&mux);
    snap = T;
    portEXIT_CRITICAL(&mux);

    if (now - lastOLED >= OLED_PERIOD_MS) {
      lastOLED = now;
      updateOLED(snap);
    }

    if (!snap.autoEnabled) {
      state = IDLE;
      setStateName("IDLE");
      motorsStop();
      turnActive = false;
      headingCorrActive = false;
      vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
      continue;
    }

    // TURNING preempts everything
    if (state == TURNING) {
      bool done = updateTurn(snap.yaw);
      if (done) {
        motorsStop();
        turnActive = false;

        // ✅ after completing a main turn, lock new forward “zero”
        resetForwardHeading(snap.yaw);

        state = turnNextState;
        setStateName(turnNextName);
      }
      vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
      continue;
    }

    // STEP execution
    if (stepActive && !stepIssued) {
      stepIssued = true;
      if (stepDir < 0) stepLeft();
      else stepRight();
      vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
      continue;
    }

    switch (state) {

      case IDLE: {
        portENTER_CRITICAL(&mux);
        T.c1 = T.c2 = T.c3 = T.c4 = 0;
        T.turnRemaining = 0;
        portEXIT_CRITICAL(&mux);

        // Default entry point
        state = Z12_FORWARD_TO_CORNER;
        setStateName("Z12_TO_CORNER");
      } break;

      // ============================================================
      // NEW ZONE 1+2 ROUTE
      // ============================================================

      // 1) Forward until corner: Front ~50±10 AND Right ~30±10
      case Z12_FORWARD_TO_CORNER: {
        setStateName("Z12_TO_CORNER");
        forwardWithHeadingHold(snap.yaw, BASE_SPEED);

        int hit = T.c1;
        bool okF = inBand(snap.f, Z12_CORNER_FRONT_CM, Z12_CORNER_FRONT_BAND);
        bool okR = inBand(snap.r, Z12_CORNER_RIGHT_CM, Z12_CORNER_RIGHT_BAND);

        if (okF && okR) hit++; else hit = 0;
        portENTER_CRITICAL(&mux); T.c1 = hit; portEXIT_CRITICAL(&mux);

        if (hit >= FRONT_REACH_CONFIRM_N) {
          motorsStop();
          portENTER_CRITICAL(&mux); T.c1 = T.c2 = T.c3 = T.c4 = 0; portEXIT_CRITICAL(&mux);
          // Turn LEFT 90°
          requestTurn(snap.yaw, -90.0f, Z12_ALIGN_RIGHT_70, "Z12_ALIGN_R70");
        }
      } break;

      // 2) Align right to 70±10
      case Z12_ALIGN_RIGHT_70: {
        setStateName("Z12_ALIGN_R70");

        if (sideIsClear(snap.r)) {
          forwardWithHeadingHold(snap.yaw, BASE_SPEED);
          break;
        }

        wallFollowRight_Target(snap.r, Z12_RIGHT_FOLLOW1_CM);

        int aCnt = T.c2;
        if (inBand(snap.r, Z12_RIGHT_FOLLOW1_CM, WALL_BAND_CM)) aCnt++; else aCnt = 0;
        portENTER_CRITICAL(&mux); T.c2 = aCnt; portEXIT_CRITICAL(&mux);

        if (aCnt >= WALL_ALIGN_CONFIRM_N) {
          portENTER_CRITICAL(&mux); T.c3 = 0; portEXIT_CRITICAL(&mux);
          state = Z12_FOLLOW_RIGHT_70;
          setStateName("Z12_FOLLOW_R70");
        }
      } break;

      // 3) Follow right @70 until right becomes clear
      case Z12_FOLLOW_RIGHT_70: {
        setStateName("Z12_FOLLOW_R70");
        wallFollowRight_Target(snap.r, Z12_RIGHT_FOLLOW1_CM);

        int clr = T.c3;
        if (sideIsClear(snap.r)) clr++; else clr = 0;
        portENTER_CRITICAL(&mux); T.c3 = clr; portEXIT_CRITICAL(&mux);

        if (clr >= SIDE_CLEAR_CONFIRM_N) {
          motorsStop();
          portENTER_CRITICAL(&mux); T.c1 = T.c2 = T.c3 = T.c4 = 0; portEXIT_CRITICAL(&mux);
          // Turn RIGHT 90°
          requestTurn(snap.yaw, +90.0f, Z12_FORWARD_TO_RIGHT_WALL_2, "Z12_TO_R_WALL2");
        }
      } break;

      // 4) Forward until right wall detected again (not clear)
      case Z12_FORWARD_TO_RIGHT_WALL_2: {
        setStateName("Z12_TO_R_WALL2");
        forwardWithHeadingHold(snap.yaw, BASE_SPEED);

        int found = T.c1;
        if (!sideIsClear(snap.r)) found++; else found = 0;
        portENTER_CRITICAL(&mux); T.c1 = found; portEXIT_CRITICAL(&mux);

        if (found >= SIDE_FOUND_CONFIRM_N) {
          motorsStop();
          portENTER_CRITICAL(&mux); T.c2 = 0; portEXIT_CRITICAL(&mux);
          state = Z12_ALIGN_RIGHT_50;
          setStateName("Z12_ALIGN_R50");
        }
      } break;

      // 5) Align right to 50±10
      case Z12_ALIGN_RIGHT_50: {
        setStateName("Z12_ALIGN_R50");

        if (sideIsClear(snap.r)) {
          forwardWithHeadingHold(snap.yaw, BASE_SPEED);
          break;
        }

        wallFollowRight_Target(snap.r, Z12_RIGHT_FOLLOW2_CM);

        int aCnt = T.c2;
        if (inBand(snap.r, Z12_RIGHT_FOLLOW2_CM, WALL_BAND_CM)) aCnt++; else aCnt = 0;
        portENTER_CRITICAL(&mux); T.c2 = aCnt; portEXIT_CRITICAL(&mux);

        if (aCnt >= WALL_ALIGN_CONFIRM_N) {
          portENTER_CRITICAL(&mux); T.c3 = 0; portEXIT_CRITICAL(&mux);
          state = Z12_FOLLOW_RIGHT_50;
          setStateName("Z12_FOLLOW_R50");
        }
      } break;

      // 6) Follow right @50 until right becomes clear, then go Zone 3
      case Z12_FOLLOW_RIGHT_50: {
        setStateName("Z12_FOLLOW_R50");
        wallFollowRight_Target(snap.r, Z12_RIGHT_FOLLOW2_CM);

        int clr = T.c3;
        if (sideIsClear(snap.r)) clr++; else clr = 0;
        portENTER_CRITICAL(&mux); T.c3 = clr; portEXIT_CRITICAL(&mux);

        if (clr >= SIDE_CLEAR_CONFIRM_N) {
          motorsStop();
          portENTER_CRITICAL(&mux); T.c1 = T.c2 = T.c3 = T.c4 = 0; portEXIT_CRITICAL(&mux);
          // ✅ Smooth transfer into Zone 3 start
          state = Z3_FORWARD_TO_FRONT;
          setStateName("Z3_FORWARD_TO_FRONT");

          // new forward leg: lock heading again
          resetForwardHeading(snap.yaw);
        }
      } break;

      // ============================================================
      // ZONE 3 (UNCHANGED)
      // ============================================================
      case Z3_FORWARD_TO_FRONT: {
        setStateName("Z3_FORWARD_TO_FRONT");
        forwardWithHeadingHold(snap.yaw, BASE_SPEED);

        int fCnt = T.c1;
        if (frontReached(snap.f, FRONT_SETPOINT_CM, FRONT_BAND_CM)) fCnt++; else fCnt = 0;
        portENTER_CRITICAL(&mux); T.c1 = fCnt; portEXIT_CRITICAL(&mux);

        if (fCnt >= FRONT_REACH_CONFIRM_N) {
          motorsStop();
          portENTER_CRITICAL(&mux); T.c2 = 0; portEXIT_CRITICAL(&mux);
          requestTurn(snap.yaw, +90.0f, Z3_ALIGN_LEFT_100, "Z3_ALIGN_LEFT_100");
        }
      } break;

      case Z3_ALIGN_LEFT_100: {
        setStateName("Z3_ALIGN_LEFT_100");
        wallFollowLeft_Default100(snap.l);

        int aCnt = T.c2;
        if (inBand(snap.l, WALL_SETPOINT_CM, WALL_BAND_CM)) aCnt++; else aCnt = 0;
        portENTER_CRITICAL(&mux); T.c2 = aCnt; portEXIT_CRITICAL(&mux);

        if (aCnt >= WALL_ALIGN_CONFIRM_N) {
          portENTER_CRITICAL(&mux); T.c3 = 0; portEXIT_CRITICAL(&mux);
          state = Z3_FOLLOW_LEFT_WALL;
          setStateName("Z3_FOLLOW_LEFT_WALL");
        }
      } break;

      case Z3_FOLLOW_LEFT_WALL: {
        setStateName("Z3_FOLLOW_LEFT_WALL");
        wallFollowLeft_Default100(snap.l);

        int clr = T.c3;
        if (sideIsClear(snap.l)) clr++; else clr = 0;
        portENTER_CRITICAL(&mux); T.c3 = clr; portEXIT_CRITICAL(&mux);

        if (clr >= SIDE_CLEAR_CONFIRM_N) {
          portENTER_CRITICAL(&mux); T.c4 = 0; portEXIT_CRITICAL(&mux);
          state = Z3_FORWARD_TO_FRONT_2;
          setStateName("Z3_FORWARD_TO_FRONT_2");

          // new forward leg: lock heading again
          resetForwardHeading(snap.yaw);
        }
      } break;

      case Z3_FORWARD_TO_FRONT_2: {
        setStateName("Z3_FORWARD_TO_FRONT_2");
        forwardWithHeadingHold(snap.yaw, BASE_SPEED);

        int fCnt = T.c4;
        if (frontReached(snap.f, FRONT_SETPOINT_CM, FRONT_BAND_CM)) fCnt++; else fCnt = 0;
        portENTER_CRITICAL(&mux); T.c4 = fCnt; portEXIT_CRITICAL(&mux);

        if (fCnt >= FRONT_REACH_CONFIRM_N) {
          motorsStop();
          portENTER_CRITICAL(&mux); T.c1 = 0; portEXIT_CRITICAL(&mux);
          requestTurn(snap.yaw, +90.0f, Z3_ALIGN_LEFT_200, "Z3_ALIGN_LEFT_200");
        }
      } break;

      case Z3_ALIGN_LEFT_200: {
        setStateName("Z3_ALIGN_LEFT_200");
        wallFollowLeft_Z3_200(snap.l);

        int aCnt = T.c1;
        if (inBand(snap.l, Z3_LEFT_ALIGN_CM, WALL_BAND_CM)) aCnt++; else aCnt = 0;
        portENTER_CRITICAL(&mux); T.c1 = aCnt; portEXIT_CRITICAL(&mux);

        if (aCnt >= WALL_ALIGN_CONFIRM_N) {
          motorsStop();
          state = Z3_DONE;
          setStateName("Z3_DONE");
        }
      } break;

      case Z3_DONE: {
        setStateName("Z3_DONE");
        motorsStop();
      } break;

      default: break;
    }

    vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

// ================================================================
// CHECKUP task
// ✅ Highest priority + direct Controller.h initialcheckuproutine()
// ================================================================
void checkupTask(void* pv) {
  (void)pv;
  for (;;) {
    if (checkupRequested && !checkupRunning) {
      checkupRequested = false;
      checkupRunning = true;
      initRoutineRunning = true;

      portENTER_CRITICAL(&mux);
      T.autoEnabled = false;
      T.turnRemaining = 0;
      strncpy(T.action, "STOP", sizeof(T.action)-1);
      T.action[sizeof(T.action)-1] = '\0';
      portEXIT_CRITICAL(&mux);

      turnActive = false;
      stepActive = false;
      stepIssued = false;
      stepDir = 0;
      headingCorrActive = false;

      state = IDLE;
      setStateName("IDLE");
      motorsStop();

      // ✅ Direct call to Controller.h routine (no re-implementation)
      if (motorMutex && xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
        stop();
        initialcheckuproutine();
        stop();
        xSemaphoreGive(motorMutex);
      } else {
        stop();
        initialcheckuproutine();
        stop();
      }

      initRoutineRunning = false;
      checkupRunning = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ---------------- Webpage ----------------
static const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ROBO CONTROL</title>
  <style>
    :root{
      --bg:#f2f3f5; --card:#ffffff; --shadow:0 6px 18px rgba(0,0,0,.08);
      --blue:#1976ff; --red:#e74c3c; --text:#111; --muted:#7b7b7b; --green:#15a85b;
    }
    body{margin:0;background:var(--bg);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;color:var(--text);}
    .wrap{max-width:430px;margin:0 auto;padding:18px 14px 30px;}
    .title{font-size:26px;font-weight:800;display:flex;align-items:center;gap:10px;margin:6px 4px 14px;}
    .card{background:var(--card);border-radius:16px;box-shadow:var(--shadow);padding:14px;margin:12px 2px;}
    .btnRow{display:flex;gap:12px;flex-wrap:wrap;}
    .btn{
      border:none;border-radius:12px;padding:14px 14px;
      font-size:16px;font-weight:700;color:#fff;flex:1;min-width:140px;
      display:flex;align-items:center;justify-content:center;gap:10px;
      box-shadow:0 6px 14px rgba(0,0,0,.10);
      cursor:pointer;
    }
    .btnBlue{background:var(--blue);}
    .btnRed{background:var(--red);min-width:100%;}
    .statusGrid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;}
    .k{font-size:12px;color:var(--muted);letter-spacing:.4px;}
    .v{font-size:20px;font-weight:800;margin-top:4px;}
    .v.green{color:var(--green);}
    .sensorsGrid{display:grid;grid-template-columns:1fr 1fr;gap:12px;}
    .sensorBox{border-radius:14px;background:#fafafa;padding:12px;border:1px solid #eee;}
    .sensorBox .k{font-size:12px;color:var(--muted);}
    .sensorBox .v{font-size:28px;font-weight:900;margin-top:6px;}
    .wpRow{display:flex;align-items:baseline;justify-content:space-between;margin-top:10px;padding-top:10px;border-top:1px solid #eee;}
    .wpRow .v{font-size:22px;}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="title">🤖 ROBO CONTROL</div>

    <div class="card">
      <div class="btnRow">
        <button class="btn btnBlue" onclick="post('/reset')">🔄 RESET</button>
        <button class="btn btnBlue" onclick="post('/go')">🏁 GO TO H</button>
        <button class="btn btnBlue" onclick="post('/checkup')">🧪 INITIAL CHECKUP</button>
      </div>
      <div style="height:12px"></div>
      <button class="btn btnRed" onclick="post('/stop')">⛔ STOP</button>
    </div>

    <div class="card">
      <div class="statusGrid">
        <div>
          <div class="k">SYSTEM STATUS</div>
          <div class="v green" id="sys">READY</div>
        </div>
        <div>
          <div class="k">MODE</div>
          <div class="v" id="mode">IDLE</div>
        </div>
        <div>
          <div class="k">GUIDANCE</div>
          <div class="v" id="guide">STOP</div>
        </div>
      </div>

      <div class="wpRow">
        <div class="k">WP PASSED</div>
        <div class="v" id="wp">0/2</div>
      </div>
    </div>

    <div class="card">
      <div class="k" style="margin-bottom:10px;">SENSORS</div>
      <div class="sensorsGrid">
        <div class="sensorBox"><div class="k">FRONT (CM)</div><div class="v" id="front">-</div></div>
        <div class="sensorBox"><div class="k">BACK (CM)</div><div class="v" id="back">-</div></div>
        <div class="sensorBox"><div class="k">LEFT (CM)</div><div class="v" id="left">-</div></div>
        <div class="sensorBox"><div class="k">RIGHT (CM)</div><div class="v" id="right">-</div></div>
      </div>
    </div>
  </div>

<script>
function fmtInt(x){
  if (x===null || x===undefined) return '-';
  return String(Math.round(x));
}
async function post(url){
  try{ await fetch(url,{method:'POST'}); }catch(e){}
}
const es = new EventSource('/events');
es.onmessage = (ev) => {
  const j = JSON.parse(ev.data);
  document.getElementById('sys').textContent = j.sys;
  document.getElementById('mode').textContent = j.mode;
  document.getElementById('guide').textContent = j.guidance;
  document.getElementById('wp').textContent = j.wp;

  document.getElementById('front').textContent = fmtInt(j.front);
  document.getElementById('back').textContent  = fmtInt(j.back);
  document.getElementById('left').textContent  = fmtInt(j.left);
  document.getElementById('right').textContent = fmtInt(j.right);
};
</script>
</body>
</html>
)HTML";

// ---------------- SSE publisher ----------------
static void pushTelemetry() {
  Telemetry snap;
  portENTER_CRITICAL(&mux);
  snap = T;
  portEXIT_CRITICAL(&mux);

  String mode = snap.autoEnabled ? String(snap.state) : "IDLE";
  String wp   = String((int)snap.wpPassed) + "/2";
  String guide = makeGuidance(snap);

  String sys = "READY";
  if (checkupRunning || initRoutineRunning) sys = "INIT CHECKUP RUNNING";

  String j;
  j.reserve(360);
  j += "{";
  j += "\"sys\":\"" + sys + "\",";
  j += "\"mode\":\"" + mode + "\",";
  j += "\"guidance\":\"" + guide + "\",";
  j += "\"wp\":\"" + wp + "\",";
  j += "\"front\":" + String(snap.f, 1) + ",";
  j += "\"back\":"  + String(snap.b, 1) + ",";
  j += "\"left\":"  + String(snap.l, 1) + ",";
  j += "\"right\":" + String(snap.r, 1);
  j += "}";

  events.send(j.c_str(), nullptr, millis());
}

// ---------------- Commands ----------------
static void startAuto() {
  if (checkupRunning || initRoutineRunning) return;

  // ✅ capture current yaw as “zero” for first forward segment
  Telemetry snap;
  portENTER_CRITICAL(&mux); snap = T; portEXIT_CRITICAL(&mux);
  resetForwardHeading(snap.yaw);

  portENTER_CRITICAL(&mux);
  T.autoEnabled = true;
  T.c1 = T.c2 = T.c3 = T.c4 = 0;
  T.turnRemaining = 0;
  strncpy(T.action, "FORWARD", sizeof(T.action)-1);
  T.action[sizeof(T.action)-1] = '\0';
  portEXIT_CRITICAL(&mux);

  turnActive = false;
  stepActive = false;
  stepIssued = false;
  stepDir = 0;
  headingCorrActive = false;

  state = Z12_FORWARD_TO_CORNER;
  setStateName("Z12_TO_CORNER");
}

static void stopAuto() {
  portENTER_CRITICAL(&mux);
  T.autoEnabled = false;
  T.turnRemaining = 0;
  portEXIT_CRITICAL(&mux);

  turnActive = false;
  stepActive = false;
  stepIssued = false;
  stepDir = 0;
  headingCorrActive = false;

  state = IDLE;
  setStateName("IDLE");
  motorsStop();
}

static void resetAll() {
  stopAuto();
  portENTER_CRITICAL(&mux);
  T.f = 251; T.b = 251; T.l = 251; T.r = 251;
  T.c1 = T.c2 = T.c3 = T.c4 = 0;
  T.turnRemaining = 0;
  T.wpPassed = 0;
  strncpy(T.action, "STOP", sizeof(T.action)-1);
  T.action[sizeof(T.action)-1] = '\0';
  portEXIT_CRITICAL(&mux);

  headingCorrActive = false;
}

static void requestCheckup() {
  checkupRequested = true;
}

// ---------------- Setup ----------------
// ✅ Add global flag
// ✅ NEW: Server initialization task
void serverTask(void* pv) {
  (void)pv;
  
  // Wait for WiFi and other tasks to stabilize
  Serial.println("Server task: Waiting for system to stabilize...");
  vTaskDelay(pdMS_TO_TICKS(3000));
  
  Serial.println("Server task: Configuring routes...");
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", PAGE);
  });
  
  server.on("/go", HTTP_POST, [](AsyncWebServerRequest* req) {
    startAuto();
    req->send(200, "text/plain", "OK");
  });
  
  server.on("/stop", HTTP_POST, [](AsyncWebServerRequest* req) {
    stopAuto();
    req->send(200, "text/plain", "OK");
  });
  
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest* req) {
    resetAll();
    req->send(200, "text/plain", "OK");
  });
  
  server.on("/checkup", HTTP_POST, [](AsyncWebServerRequest* req) {
    requestCheckup();
    req->send(200, "text/plain", "OK");
  });

  Serial.println("Server task: Configuring SSE...");
  
  events.onConnect([](AsyncEventSourceClient* client) {
    if (client) {
      client->send("{\"hello\":true}", nullptr, millis());
    }
  });
  
  server.addHandler(&events);
  
  Serial.println("Server task: Starting server...");
  server.begin();
  
  serverStarted = true;
  Serial.println("Server task: ✅ WEB SERVER RUNNING!");
  
  // Update OLED
  if (i2cMutex && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("READY");
    display.setCursor(0, 16);
    display.print("AP: team4");
    display.display();
    xSemaphoreGive(i2cMutex);
  }
  
  // Task is done - delete itself
  vTaskDelete(NULL);
}
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n=== TEAM4 ROBO CONTROL STARTING ===");

  // 1) Initialize I2C
  Wire.begin(21, 22);
  delay(100);
  i2cMutex = xSemaphoreCreateMutex();
  Serial.println("I2C initialized");

  // 2) Initialize Controller hardware (continue even if failed)
  Serial.println("Initializing DAC...");
  bool dacOk = initDAC();
  Serial.println(dacOk ? "DAC OK" : "⚠️ DAC FAILED - Motor control disabled");
  
  Serial.println("Initializing Expander...");
  bool expOk = initExpMod();
  Serial.println(expOk ? "Expander OK" : "⚠️ Expander FAILED - Button control disabled");
  
  if (dacOk && expOk) {
    stop();
  }

  // 3) Initialize OLED
  if (i2cMutex && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    bool ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    if (ok) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.print("ROBO CONTROL");
      display.setCursor(0, 16);
      display.print("Starting WiFi...");
      display.display();
    }
    Serial.println(ok ? "OLED OK" : "OLED FAILED");
    xSemaphoreGive(i2cMutex);
  }

  // 4) Initialize IMU
  if (i2cMutex && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    bool imuOK = bno.begin();
    if (imuOK) {
      bno.setExtCrystalUse(true);
      Serial.println("IMU OK");
    } else {
      Serial.println("IMU FAILED");
    }
    xSemaphoreGive(i2cMutex);
  }

  // 5) Initialize GPS
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS initialized");

  // 6) Initialize Ultrasonic sensors
  usFront.begin();
  usBack.begin();
  usLeft.begin();
  usRight.begin();
  Serial.println("Ultrasonic sensors initialized");

  // 7) Motor mutex
  motorMutex = xSemaphoreCreateMutex();
  Serial.println("Motor mutex created");

  // ✅ 8) WiFi setup
  Serial.println("Starting WiFi AP...");
  WiFi.mode(WIFI_AP);
  delay(100);
  
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(1000);
  
  Serial.print("✅ AP Started - IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("   SSID: team4");
  Serial.println("   Password: 12345678");

  // ✅ 9) Create all tasks (including server task)
  Serial.println("Creating tasks...");
  
  xTaskCreatePinnedToCore(gpsTask,     "gpsTask",     4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(sensorTask,  "sensorTask",  4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(controlTask, "controlTask", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(checkupTask, "checkupTask", 6144, nullptr, 3, nullptr, 0);
  
  // ✅ Server task with high stack size
  xTaskCreatePinnedToCore(serverTask,  "serverTask",  8192, nullptr, 1, nullptr, 0);
  
  Serial.println("✅ All tasks created (server will start in 3 seconds)");
  Serial.println("=== INITIALIZATION COMPLETE ===\n");
}

void loop() {
  static uint32_t lastPush = 0;
  uint32_t now = millis();
  
  // ✅ Only send SSE after server is ready
  if (serverStarted && (now - lastPush >= SSE_PERIOD_MS)) {
    lastPush = now;
    pushTelemetry();
  }
  
  // Small delay to prevent watchdog
  delay(1);
}


