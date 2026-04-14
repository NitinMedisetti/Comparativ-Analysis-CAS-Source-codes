/*
  =================== SINGLE INTEGRATED SKETCH ===================
  FIX ADDED:
  ✅ Web "/dance" no longer runs dance inside the AsyncWebServer callback (prevents crash/reset)
  ✅ Dance is executed inside a dedicated FreeRTOS task (taskDance)
  ✅ Added systemPaused flag so tasks pause safely (instead of suspending mid-I2C)
  ✅ Added reset reason print (esp_reset_reason)
  ✅ OPTIONAL SAFETY (ADDED NOW): taskNav() will force STOP if danceRequested or danceRunning

  ✅ OBSTACLE RULES (ADDED NOW):
  - If Front + Right covered -> STEP LEFT
  - If Front + Left covered  -> STEP RIGHT

  ✅ NEW FIXES (ADDED NOW):
  - FIX A: taskWeb() no longer pauses SSE during systemPaused; it continues sending "DANCE" updates
           (OLED rendering is skipped while paused to avoid I2C contention)
  - FIX B: DANCE task moved off WiFi core (core 0) -> now pinned to core 1, lower priority to prevent AP reset

  ✅ NEW ADD (NOW):
  - FINAL WAYPOINT (PHASE_H_WP4): if RIGHT ultrasonic detects obstacle continuously for 6s,
    force COMPLETE + stop nav + execute laydown via PHASE_COMPLETE.

  ✅ CHANGE (NOW):
  - Removed the "IMU Yaw Offset" buttons/controls from the WEBSITE (UI).
    (Firmware offset logic remains unchanged.)

  ✅ NEW CHANGE (YOUR REQUEST NOW):
  - ONLY for PHASE_H_WP2 (WP1 -> WP2 navigation):
      If FRONT detects obstacle:
        1) First time: STEP LEFT for 4 seconds
        2) After that (same phase): ONLY STEP RIGHT for 4 seconds
      (This does not affect other phases.)
*/

#include <WiFi.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_SSD1306.h>

// #include "Controller.h" // ✅ keep unchanged

/* ================= CONFIG ================= */
#define SDA_PIN 21
#define SCL_PIN 22

#define GPS_RX 16
#define GPS_TX 17

#define GPS_POINTS_REQUIRED 50
#define GPS_MIN_SPEED_MPS 0.1f
#define ALIGN_TOL_DEG 5.0f

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

#define GPS_THRESHOLD 1.0f
#define OBSTACLE_CM 60.0f

/* ================= GLOBALS ================= */
Adafruit_BNO055 bno(55, 0x28, &Wire);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

AsyncWebServer server(80);
AsyncEventSource events("/events");

/* ================= TASK HANDLES ================= */
static TaskHandle_t hIMU   = nullptr;
static TaskHandle_t hGPS   = nullptr;
static TaskHandle_t hFUS   = nullptr;
static TaskHandle_t hUS    = nullptr;
static TaskHandle_t hCALM  = nullptr;
static TaskHandle_t hNAV   = nullptr;
static TaskHandle_t hWEB   = nullptr;
static TaskHandle_t hDANCE = nullptr;

// ✅ pause flag (avoid vTaskSuspend mid-I2C)
static volatile bool systemPaused = false;

/* ================= IMU OFFSET ================= */
static volatile int imuOffsetDeg = -180;

static bool isAllowedOffset(int deg) {
  return (deg == 0 ||
          deg == 90   || deg == -90  ||
          deg == 180  || deg == -180 ||
          deg == 270  || deg == -270);
}

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* ================= STATE ================= */
// IMU
static float imuYawRaw = 0.0f;
static float imuYaw    = 0.0f;
static bool imuOk = false;

// True North lock
static float trueNorthOffset = 0.0f;
static bool trueNorthLocked = false;

// GPS course accumulation
static float sumSin = 0.0f, sumCos = 0.0f;
static int gpsCount = 0;

// Current GPS position
struct GPSCoordinate { double lat; double lon; };
static GPSCoordinate currentPoint = {0.0, 0.0};

// Navigation targets
static GPSCoordinate prePhasePoint = {48.829985, 12.954892}; // POST
static GPSCoordinate waypoint0_H = {48.829845, 12.955100};
static GPSCoordinate waypoint1   = {48.829798, 12.954993};
static GPSCoordinate waypoint2   = {48.829549, 12.954733};
static GPSCoordinate waypoint3   = {48.829610, 12.954389};
static GPSCoordinate waypoint4   = {48.829492, 12.954299};

static float targetBearing = 0.0f;
static float distanceToTarget = 0.0f;

// Ultrasonic
struct UltrasonicSensor { int trigPin; int echoPin; const char* name; };
UltrasonicSensor sensors[5] = {
  {23, 26, "45"},
  {32, 25, "Front"},
  {5, 14, "Back"},
  {4, 18, "Left"},
  {13, 19, "Right"}
};
static float ultrasonicDistances[5] = {0,0,0,0,0};

// Control flags
static bool navEnabled = false;
static bool calibrating = false;
static bool endLaydownDone = false;

// Navigation phases
enum NavigationPhase {
  PHASE_IDLE = 0,
  PHASE_GOTO_POST,
  PHASE_H_WP0,
  PHASE_H_WP1,
  PHASE_H_WP2,
  PHASE_H_WP3,   // ✅ WP2 -> WP3 happens during this phase
  PHASE_H_WP4,
  PHASE_COMPLETE
};
static NavigationPhase currentPhase = PHASE_IDLE;

// Commands
static const char* navCmd   = "STOP";
static const char* calibCmd = "IDLE";

// ✅ DANCE flags
static volatile bool danceRequested = false;
static volatile bool danceRunning   = false;

// ✅ NEW: RIGHT ultrasonic hold timer for final waypoint completion
static uint32_t rightHoldStartMs = 0;

// ✅ NEW (YOUR REQUEST): WP1->WP2 special logic flag (PHASE_H_WP2)
static bool wp1to2_leftDone = false;

/* ================= UTILS ================= */
static float wrap360(float a) {
  while (a < 0) a += 360;
  while (a >= 360) a -= 360;
  return a;
}
static float wrap180(float a) {
  while (a < -180) a += 360;
  while (a > 180) a -= 360;
  return a;
}
static float fusedHeadingTrue() {
  float h = imuYaw;
  if (trueNorthLocked) h = wrap360(imuYaw + trueNorthOffset);
  return h;
}

/* ================= GEO ================= */
static float calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  float R = 6371000;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}
static float calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) -
            sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  float bearing = degrees(atan2(y, x));
  bearing = fmod((bearing + 360), 360);
  return bearing;
}

/* ================= ULTRASONIC ================= */
static float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  float cm = duration * 0.034f / 2.0f;
  return cm;
}

/* ================= GO2 EXECUTION ================= */
static void stopAllMotion() { 
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("stop()");
  display.display();
  // stop(); 
}

static void applyCmdToGo2(const char* cmd) {
  if (!strcmp(cmd, "FORWARD")) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("forward(28)");
    display.display();
    // forward(percent_routine);
  }
  else if (!strcmp(cmd, "TURN RIGHT")) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("rotateRight(28)");
    display.display();
    // rotateRight(percent_routine);
  }
  else if (!strcmp(cmd, "TURN LEFT")) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("rotateLeft(28)");
    display.display();
    // rotateLeft(percent_routine);
  }
  else stopAllMotion();
}

/* ================= OBSTACLE AVOIDANCE ================= */
static bool obstacleAvoidanceActive() {
  float f = ultrasonicDistances[1]; // Front
  float l = ultrasonicDistances[3]; // Left
  float r = ultrasonicDistances[4]; // Right

  // Treat 0 as invalid / no reading
  bool fHit = (f > 0.0f && f < OBSTACLE_CM);
  bool lHit = (l > 0.0f && l < OBSTACLE_CM);
  bool rHit = (r > 0.0f && r < OBSTACLE_CM);

  // ✅ NEW (YOUR REQUEST):
  // ONLY for PHASE_H_WP2 (WP1 -> WP2 navigation):
  //   - If FRONT hits obstacle:
  //       first time -> STEP LEFT for 4s
  //       afterwards (same phase) -> ONLY STEP RIGHT for 4s
  if (currentPhase == PHASE_H_WP2 && fHit) {
    if (!wp1to2_leftDone) {
      navCmd = "WP1->2 FRONT -> STEP LEFT(4s)";
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println("stepLeft(28)");
      display.display();
      // stepLeft(percent_routine);
      vTaskDelay(pdMS_TO_TICKS(4000));
      stopAllMotion();
      wp1to2_leftDone = true;
    } else {
      navCmd = "WP1->2 FRONT -> STEP RIGHT(4s)";
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println("stepRight(28)");
      display.display();
      // stepRight(percent_routine);
      vTaskDelay(pdMS_TO_TICKS(4000));
      stopAllMotion();
    }
    return true;
  }

  // ✅ Combo priority rules (general)
  // Front + Right covered -> STEP LEFT
  if (fHit && rHit) {
    navCmd = "FR+R -> STEP LEFT";
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("stepLeft(28)");
    display.display();
    // stepLeft(percent_routine);
    return true;
  }
  // Front + Left covered  -> STEP RIGHT
  if (fHit && lHit) {
    navCmd = "FR+L -> STEP RIGHT";
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("stepRight(28)");
    display.display();
    // stepRight(percent_routine);
    return true;
  }

  // Fallback single-sensor rules
  if (fHit) { 
    navCmd = "FRONT -> STEP LEFT";
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("stepLeft(28)");
    display.display();
    // stepLeft(percent_routine);
    return true; 
  }
  if (rHit) { 
    navCmd = "RIGHT -> STEP LEFT";
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("stepLeft(28)");
    display.display();
    // stepLeft(percent_routine);
    return true; 
  }
  if (lHit) { 
    navCmd = "LEFT -> STEP RIGHT";
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("stepRight(28)");
    display.display();
    // stepRight(percent_routine);
    return true; 
  }

  return false;
}

/* ================= TASK: IMU ================= */
void taskIMU(void*) {
  for (;;) {
    if (systemPaused) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    if (imuOk) {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      imuYawRaw = wrap360(euler.x());
      imuYaw = wrap360(imuYawRaw + (float)imuOffsetDeg);
    } else {
      imuYawRaw = 0.0f;
      imuYaw = 0.0f;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* ================= TASK: GPS ================= */
void taskGPS(void*) {
  for (;;) {
    if (systemPaused) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    while (gpsSerial.available()) gps.encode(gpsSerial.read());

    if (gps.location.isValid()) {
      currentPoint.lat = gps.location.lat();
      currentPoint.lon = gps.location.lng();
    }

    if (calibrating && !trueNorthLocked &&
        gps.location.isValid() && gps.speed.isValid() && gps.course.isValid() &&
        (gps.speed.mps() > GPS_MIN_SPEED_MPS)) {
      float c = gps.course.deg();
      sumCos += cos(radians(c));
      sumSin += sin(radians(c));
      gpsCount++;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* ================= TASK: FUSION ================= */
void taskFusion(void*) {
  for (;;) {
    if (systemPaused) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    if (calibrating && !trueNorthLocked && gpsCount >= GPS_POINTS_REQUIRED) {
      float avgCourse = wrap360(degrees(atan2(sumSin, sumCos)));
      trueNorthOffset = wrap180(avgCourse - imuYaw);
      trueNorthLocked = true;
      Serial.println("TRUE NORTH LOCKED");
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

/* ================= TASK: ULTRASONIC ================= */
void taskUltrasonic(void*) {
  for (;;) {
    if (systemPaused) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    for (int i = 0; i < 5; i++) {
      ultrasonicDistances[i] = measureDistance(sensors[i].trigPin, sensors[i].echoPin);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(80));
  }
}

/* ================= CALIBRATION MOTION ================= */
void taskCalibrationMotion(void*) {
  for (;;) {
    if (systemPaused) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    if (calibrating) {
      float trueH = fusedHeadingTrue();

      if (!trueNorthLocked) {
        calibCmd = "MOVE FOR CALIBRATION";
        applyCmdToGo2("FORWARD");
      } else {
        float err = wrap180(0.0f - trueH);
        if (fabs(err) <= ALIGN_TOL_DEG) {
          calibCmd = "ALIGNED";
          stopAllMotion();
          calibrating = false;
          calibCmd = "DONE";
        } else if (err > 0) {
          calibCmd = "TURN RIGHT";
          applyCmdToGo2("TURN RIGHT");
        } else {
          calibCmd = "TURN LEFT";
          applyCmdToGo2("TURN LEFT");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(120));
  }
}

/* ================= NAVIGATION ================= */
static void computeGPSNavCommand(double tgtLat, double tgtLon) {
  if (currentPoint.lat == 0.0 || currentPoint.lon == 0.0) { navCmd = "WAIT GPS"; return; }

  distanceToTarget = calculateDistance(currentPoint.lat, currentPoint.lon, tgtLat, tgtLon);
  targetBearing    = calculateBearing (currentPoint.lat, currentPoint.lon, tgtLat, tgtLon);

  float h = fusedHeadingTrue();
  float diff = wrap180(targetBearing - h);

  if (distanceToTarget <= GPS_THRESHOLD) { navCmd = "ARRIVED"; return; }

  if (fabs(diff) < 15) navCmd = "FORWARD";
  else if (diff > 0)   navCmd = "TURN RIGHT";
  else                 navCmd = "TURN LEFT";
}

void taskNav(void*) {
  // ✅ NEW: phase-change tracking to reset WP1->WP2 special flag correctly
  static NavigationPhase prevPhase = PHASE_IDLE;

  for (;;) {
    if (systemPaused) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    // ✅ Reset WP1->WP2 special flag on phase changes (enter/leave)
    if (currentPhase != prevPhase) {
      if (currentPhase == PHASE_H_WP2) {
        wp1to2_leftDone = false; // entering WP1->WP2 phase
      } else if (prevPhase == PHASE_H_WP2) {
        wp1to2_leftDone = false; // leaving WP1->WP2 phase
      }
      prevPhase = currentPhase;
    }

    // ✅ OPTIONAL SAFETY (ADDED):
    if (danceRequested || danceRunning) {
      stopAllMotion();
      navCmd = "STOP(DANCE)";
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (calibrating) { vTaskDelay(pdMS_TO_TICKS(120)); continue; }

    if (!navEnabled) {
      stopAllMotion();
      navCmd = "STOP";
      vTaskDelay(pdMS_TO_TICKS(120));
      continue;
    }

    switch (currentPhase) {
      case PHASE_GOTO_POST:
        if (obstacleAvoidanceActive()) break;
        computeGPSNavCommand(prePhasePoint.lat, prePhasePoint.lon);
        if (!strcmp(navCmd, "ARRIVED")) { stopAllMotion(); navEnabled = false; currentPhase = PHASE_IDLE; }
        else applyCmdToGo2(navCmd);
        break;

      case PHASE_H_WP0:
        if (obstacleAvoidanceActive()) break;
        computeGPSNavCommand(waypoint0_H.lat, waypoint0_H.lon);
        if (!strcmp(navCmd, "ARRIVED")) currentPhase = PHASE_H_WP1;
        else applyCmdToGo2(navCmd);
        break;

      case PHASE_H_WP1:
        if (obstacleAvoidanceActive()) break;
        computeGPSNavCommand(waypoint1.lat, waypoint1.lon);
        if (!strcmp(navCmd, "ARRIVED")) currentPhase = PHASE_H_WP2;
        else applyCmdToGo2(navCmd);
        break;

      case PHASE_H_WP2:
        if (obstacleAvoidanceActive()) break;
        computeGPSNavCommand(waypoint2.lat, waypoint2.lon);
        if (!strcmp(navCmd, "ARRIVED")) currentPhase = PHASE_H_WP3;
        else applyCmdToGo2(navCmd);
        break;

      case PHASE_H_WP3:
        if (obstacleAvoidanceActive()) break;
        computeGPSNavCommand(waypoint3.lat, waypoint3.lon);
        if (!strcmp(navCmd, "ARRIVED")) currentPhase = PHASE_H_WP4;
        else applyCmdToGo2(navCmd);
        break;

      case PHASE_H_WP4: {
        // ✅ FINAL WAYPOINT ADD:
        // If RIGHT ultrasonic detects obstacle continuously for 6 seconds -> COMPLETE + stop + laydown
        float r = ultrasonicDistances[4]; // Right
        bool rHit = (r > 0.0f && r < OBSTACLE_CM);

        // ✅ NEW (ONLY CHANGE YOU REQUESTED):
        // If FRONT ultrasonic detects any obstacle -> COMPLETE + stop + laydown
        float f = ultrasonicDistances[1]; // Front
        bool fHit = (f > 0.0f && f < OBSTACLE_CM);
        if (fHit) {
          stopAllMotion();
          navCmd = "COMPLETE(FRONT)";
          currentPhase = PHASE_COMPLETE;
          break;
        }

        if (rHit) {
          if (rightHoldStartMs == 0) rightHoldStartMs = millis();
          if ((millis() - rightHoldStartMs) >= 6000) {
            stopAllMotion();
            navCmd = "COMPLETE(RIGHT 6s)";
            currentPhase = PHASE_COMPLETE;
            break;
          }
        } else {
          rightHoldStartMs = 0;
        }

        if (obstacleAvoidanceActive()) break;
        computeGPSNavCommand(waypoint4.lat, waypoint4.lon);
        if (!strcmp(navCmd, "ARRIVED")) currentPhase = PHASE_COMPLETE;
        else applyCmdToGo2(navCmd);
        break;
      }

      case PHASE_COMPLETE:
        rightHoldStartMs = 0; // ✅ reset the 6s trigger
        stopAllMotion();
        navCmd = "COMPLETE";
        if (!endLaydownDone) { 
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(0, 0);
          display.println("lockLaydownStand()");
          display.display();
          // lockLaydownStand();
          endLaydownDone = true; 
        }
        navEnabled = false;
        currentPhase = PHASE_IDLE;
        break;

      default:
        stopAllMotion();
        navCmd = "STOP";
        currentPhase = PHASE_IDLE;
        navEnabled = false;
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(120));
  }
}

/* ================= WEB + OLED ================= */
static void oledRender(const char* topCmd, const char* mode) {
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(0,0);
  display.println(topCmd);

  display.setTextSize(1);
  display.setCursor(0,20);
  display.print("Mode: "); display.println(mode);

  display.print("YawU:"); display.print(imuYaw, 0);
  display.print(" Off:"); display.println((int)imuOffsetDeg);

  display.print("YawR:"); display.println(imuYawRaw, 0);

  display.print("T:"); display.print(fusedHeadingTrue(), 0);
  display.print(trueNorthLocked ? " L" : " U");
  display.println();

  display.display();
}

/* ✅ FIX A: WEB task keeps SSE running during dance; OLED skipped while paused */
void taskWeb(void*) {
  for (;;) {
    // Do not pause SSE updates during systemPaused.
    // Only skip OLED I2C while paused to avoid I2C contention.
    const bool paused = systemPaused;

    const char* mode   = danceRunning ? "DANCE" : (calibrating ? "CALIB" : (navEnabled ? "NAV" : "IDLE"));
    const char* cmdOut = danceRunning ? "DANCE" : (calibrating ? calibCmd : (navEnabled ? navCmd : "STOP"));
    float tH = fusedHeadingTrue();

    char json[520];
    snprintf(json, sizeof(json),
      "{\"imuYaw\":%.2f,\"imuYawRaw\":%.2f,\"imuOffset\":%d,\"trueHeading\":%.2f,"
      "\"gpsCount\":%d,\"locked\":%s,\"mode\":\"%s\",\"cmd\":\"%s\","
      "\"lat\":%.6f,\"lon\":%.6f,\"dist\":%.2f,\"bear\":%.1f,"
      "\"usFront\":%.1f,\"usLeft\":%.1f,\"usRight\":%.1f,"
      "\"gpsSpeed\":%.2f}",
      imuYaw, imuYawRaw, (int)imuOffsetDeg, tH,
      gpsCount, trueNorthLocked ? "true":"false",
      mode, cmdOut,
      currentPoint.lat, currentPoint.lon, distanceToTarget, targetBearing,
      ultrasonicDistances[1], ultrasonicDistances[3], ultrasonicDistances[4],
      gps.speed.isValid() ? gps.speed.mps() : 0.0
    );

    events.send(json, "nav", millis());

    // Skip OLED while paused (dance) to avoid I2C contention
    if (!paused) {
      oledRender(cmdOut, mode);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

/* ================= TASK: DANCE ================= */
void taskDance(void*) {
  for (;;) {
    if (danceRequested && !danceRunning) {
      danceRequested = false;
      danceRunning = true;

      calibrating = false;
      navEnabled  = false;
      endLaydownDone = false;
      currentPhase = PHASE_IDLE;

      // ✅ reset WP1->WP2 special flag as well
      wp1to2_leftDone = false;

      navCmd = "DANCE";
      calibCmd = "IDLE";
      stopAllMotion();

      // Pause I2C-heavy tasks; WEB continues SSE (OLED skipped while paused)
      systemPaused = true;
      vTaskDelay(pdMS_TO_TICKS(200));

      // Blocking routine from Controller.h
      initialcheckuproutine();

      systemPaused = false;
      stopAllMotion();

      navCmd = "STOP";        // ✅ comes back to STOP after dance
      danceRunning = false;   // ✅ WEB will show STOP again
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/* ================= HTML UI ================= */
/* ✅ IMU Offset controls REMOVED from website */
const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 NAV</title>
  <style>
    body{font-family:Arial;margin:0;padding:16px;background:#0f172a;color:#fff}
    .card{background:#1e293b;border:2px solid #334155;border-radius:12px;padding:14px;margin-bottom:12px}
    .big{font-size:44px;text-align:center;padding:18px;background:#3b82f6;border-radius:12px}
    .row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #334155}
    .lbl{color:#94a3b8}
    .val{font-weight:700}
    .grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}
    button{padding:14px;border-radius:12px;border:2px solid #334155;background:#111827;color:#fff;font-weight:800}
    button.primary{background:#22c55e}
    button.danger{background:#ef4444}
    button.warn{background:#f59e0b;color:#111827}
  </style>
</head>
<body>
  <div class="card"><h2 style="margin:0;text-align:center">Go2 Navigation</h2></div>
  <div id="cmd" class="big">STOP</div>

  <div class="card">
    <h3 style="margin-top:0">Controls</h3>
    <div class="grid">
      <button class="warn" onclick="hit('/calibrate')">CALIBRATE</button>
      <button class="primary" onclick="hit('/goToPost')">GO TO POST</button>
      <button class="primary" onclick="hit('/goToH')">GO TO H</button>
      <button onclick="hit('/dance')">DANCE</button>
      <button onclick="hit('/laytoggle')">LAY TOGGLE</button>
      <button class="danger" onclick="hit('/stop')">STOP</button>
    </div>
  </div>

  <div class="card">
    <h3 style="margin-top:0">Telemetry</h3>
    <div id="t"></div>
  </div>

<script>
  function hit(p){ fetch(p); }

  const t=document.getElementById('t');
  const cmd=document.getElementById('cmd');
  const es=new EventSource("/events");

  es.addEventListener("nav",e=>{
    const d=JSON.parse(e.data);
    cmd.textContent=d.cmd;

    let h='';
    h+=`<div class="row"><span class="lbl">Mode</span><span class="val">${d.mode}</span></div>`;
    h+=`<div class="row"><span class="lbl">Locked</span><span class="val">${d.locked?'YES':'NO'}</span></div>`;
    h+=`<div class="row"><span class="lbl">IMU Offset</span><span class="val">${d.imuOffset}°</span></div>`;
    h+=`<div class="row"><span class="lbl">IMU Yaw (used)</span><span class="val">${d.imuYaw.toFixed(1)}°</span></div>`;
    h+=`<div class="row"><span class="lbl">IMU Yaw (raw)</span><span class="val">${d.imuYawRaw.toFixed(1)}°</span></div>`;
    h+=`<div class="row"><span class="lbl">True Heading</span><span class="val">${d.trueHeading.toFixed(1)}°</span></div>`;
    h+=`<div class="row"><span class="lbl">GPS Samples</span><span class="val">${d.gpsCount}</span></div>`;
    h+=`<div class="row"><span class="lbl">GPS Speed</span><span class="val">${d.gpsSpeed.toFixed(2)} m/s</span></div>`;
    h+=`<div class="row"><span class="lbl">GPS</span><span class="val">${d.lat.toFixed(6)}, ${d.lon.toFixed(6)}</span></div>`;
    h+=`<div class="row"><span class="lbl">Dist/Bearing</span><span class="val">${d.dist.toFixed(1)} m / ${d.bear.toFixed(0)}°</span></div>`;
    h+=`<div class="row"><span class="lbl">US Front</span><span class="val">${d.usFront.toFixed(1)} cm</span></div>`;
    h+=`<div class="row"><span class="lbl">US Left / Right</span><span class="val">${d.usLeft.toFixed(1)} / ${d.usRight.toFixed(1)} cm</span></div>`;
    t.innerHTML=h;
  });
</script>
</body>
</html>
)HTML";

/* ================= HELPERS ================= */
static void resetTrueNorthAccum() {
  sumSin = 0.0f;
  sumCos = 0.0f;
  gpsCount = 0;
  trueNorthLocked = false;
  trueNorthOffset = 0.0f;
  calibCmd = "IDLE";
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  Serial.printf("Reset reason: %d\n", (int)esp_reset_reason());

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 fail");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Boot...");
  display.display();

  // bool expOk = initExpMod();
  // bool dacOk = initDAC();
  // Serial.printf("Controller: exp=%d dac=%d\n", expOk, dacOk);
  // stopAllMotion();

  imuOk = bno.begin();
  if (imuOk) bno.setExtCrystalUse(true);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  for (int i = 0; i < 5; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32-NAV-GO2", "12345678");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){
    r->send_P(200, "text/html", PAGE);
  });

  // NOTE: Endpoint kept (UI removed only). If you also want this DISABLED, tell me.
  server.on("/setOffset", HTTP_GET, [](AsyncWebServerRequest *r){
    if (!r->hasParam("deg")) { r->send(400, "text/plain", "ERR:missing deg"); return; }
    int deg = r->getParam("deg")->value().toInt();
    if (!isAllowedOffset(deg)) { r->send(400, "text/plain", "ERR:allowed 0,90,-90,180,-180,270,-270"); return; }
    imuOffsetDeg = deg;
    r->send(200, "text/plain", "OK:OFFSET");
  });

  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *r){
    navEnabled = false;
    endLaydownDone = false;
    currentPhase = PHASE_IDLE;
    wp1to2_leftDone = false;   // ✅ reset special phase rule
    resetTrueNorthAccum();
    calibrating = true;
    r->send(200, "text/plain", "OK:CALIBRATE");
  });

  server.on("/goToPost", HTTP_GET, [](AsyncWebServerRequest *r){
    calibrating = false;
    endLaydownDone = false;
    navEnabled = true;
    currentPhase = PHASE_GOTO_POST;
    wp1to2_leftDone = false;   // ✅ reset special phase rule
    r->send(200, "text/plain", "OK:GOTO_POST");
  });

  server.on("/goToH", HTTP_GET, [](AsyncWebServerRequest *r){
    calibrating = false;
    endLaydownDone = false;
    navEnabled = true;
    currentPhase = PHASE_H_WP0;
    wp1to2_leftDone = false;   // ✅ reset special phase rule
    r->send(200, "text/plain", "OK:GO_H");
  });

  // ✅ FIXED dance: request only
  server.on("/dance", HTTP_GET, [](AsyncWebServerRequest *r){
    danceRequested = true;
    r->send(200, "text/plain", "OK:DANCE_REQUESTED");
  });

  server.on("/laytoggle", HTTP_GET, [](AsyncWebServerRequest *r){
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("lockLaydownStand()");
    display.display();
    // lockLaydownStand();
    r->send(200, "text/plain", "OK:LAY_TOGGLE");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *r){
    calibrating = false;
    navEnabled = false;
    endLaydownDone = false;
    currentPhase = PHASE_IDLE;
    wp1to2_leftDone = false;   // ✅ reset special phase rule
    stopAllMotion();
    r->send(200, "text/plain", "OK:STOP");
  });

  server.addHandler(&events);
  server.begin();

  xTaskCreatePinnedToCore(taskIMU, "IMU", 4096, NULL, 3, &hIMU, 1);
  xTaskCreatePinnedToCore(taskGPS, "GPS", 4096, NULL, 2, &hGPS, 1);
  xTaskCreatePinnedToCore(taskFusion, "FUSION", 4096, NULL, 2, &hFUS, 0);
  xTaskCreatePinnedToCore(taskUltrasonic, "US", 4096, NULL, 1, &hUS, 0);
  xTaskCreatePinnedToCore(taskCalibrationMotion, "CALM", 4096, NULL, 2, &hCALM, 0);
  xTaskCreatePinnedToCore(taskNav, "NAV", 4096, NULL, 2, &hNAV, 0);
  xTaskCreatePinnedToCore(taskWeb, "WEB", 4096, NULL, 1, &hWEB, 0);

  // ✅ FIX B: Dance task moved off WiFi core (core 0) -> core 1, lower priority
  xTaskCreatePinnedToCore(taskDance, "DANCE", 8192, NULL, 1, &hDANCE, 1);
}

void loop() {
  // All logic runs in tasks
}