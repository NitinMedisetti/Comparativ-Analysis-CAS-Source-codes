#define robot 
#define x_off 0.0
#define y_off 0.0
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#include <Adafruit_MCP4728.h>
#include <Adafruit_MCP23X17.h>

#include <math.h>

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <cmath>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>

#include <NewPing.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <esp_now.h>
#include <Adafruit_BNO055.h>

#include "esp_system.h"
#include "esp_task_wdt.h"

#include "NavTypes.h"
#include "LD20_Lidar.h"
#include "wall_angle_calculator.h"
#include "GPS_Module.h"
#include "UltrasonicSensors.h"
#include "BNO055Sensor.h"
#include "webpage.h"
#include "OLED_Display.h"
#include "path.h"
// #include "Controller.h"  // No longer needed: expMod and DAC removed from circuit

volatile bool checkupRequested = false;

// --- Configuration ---
const int SERIAL_DEBUG_BAUD = 115200;
#define LIDAR_QUEUE_SIZE 8
#define GPS_QUEUE_SIZE 1
#define WEBSOCKET_RATE_MS 100  // 100 Hz
#define LD_RATE 1
#define FULL_SCAN_POINTS 360             // Assuming 360 points in the aggregated frame
const float DESIRED_WALL_ANGLE = 90.0f;  // Target wall angle for parallel travel

// --- Global Hardware and Communication Objects ---
WebServer server(80);
WebSocketsServer webSocket(81);
// Note: Pins 27, 33 are passed to lidar.begin() in lidarRxTask
LD20_Lidar lidar(&Serial2);
GPSModule gps;
bool Robot_connected = false ;
bool Robot_laydown = false ;

static SemaphoreHandle_t cmdMutex = NULL;
static char web_cmd[16] = "STOP";

static inline void setCmd(const char *c) {
  if (xSemaphoreTake(cmdMutex, pdMS_TO_TICKS(2)) == pdPASS) {
    strlcpy(web_cmd, c, sizeof(web_cmd));
    xSemaphoreGive(cmdMutex);
  }
}
static inline void getCmd(char *out, size_t n) {
  if (xSemaphoreTake(cmdMutex, pdMS_TO_TICKS(2)) == pdPASS) {
    strlcpy(out, web_cmd, n);
    xSemaphoreGive(cmdMutex);
  } else {
    strlcpy(out, "NA", n);
  }
}


SemaphoreHandle_t sensorMutex = NULL;
// Protects slower, computationally intensive Lidar angle data
SemaphoreHandle_t lidarAngleMutex = NULL;


QueueHandle_t gpsDataQueue = NULL;
QueueHandle_t lidarDataQueue = NULL;

TaskHandle_t pathToHHandle = NULL;
TaskHandle_t pathToPostHandle = NULL;

TaskHandle_t hRobotControl   = NULL;
TaskHandle_t hWebServer      = NULL;
TaskHandle_t hDataCombine    = NULL;
TaskHandle_t hIMU_US         = NULL;
TaskHandle_t hGPS            = NULL;
TaskHandle_t hLidarRx        = NULL;
TaskHandle_t hLidarProc      = NULL;
TaskHandle_t hOLED_Update    = NULL;

static char wsMsg[512];


// --- Global Shared Data (Protected by their respective Mutexes) ---
// Protected by lidarAngleMutex
AggregatedLidarFrame fullScanFrame = { 0 };
float currentleftWallAngle = 999.0f;
float currentleftWallDistance;
float currentrightWallAngle = 999.0f;
float currentrightWallDistance;
float currentfrontWallAngle = 999.0f;
float currentfrontWallDistance;
float cor_wall = 999.0f;  // Angular correction needed (Lidar calculated)




IMU_US_Data currentSensorData = { 0 };



// Lidar frame wrapper (for queue transport)
struct LidarFrame {
  LidarPoint points[LD20_Lidar::POINTS_PER_FRAME];
};





static const int NUM_WP = 5; 
static const double WP_LAT[NUM_WP] = {48.829845, 48.829798, 48.829549, 48.829610, 48.829492};
static const double WP_LON[NUM_WP] = {12.955100, 12.954993, 12.954733, 12.954389, 12.954299};
static const double WP_LAT_1 = 48.830116 ;
static const double WP_LON_1 = 12.955072 ;
// --- Globals for OLED/Web display (Protected by sensorMutex) ---
float currentHeading = 0.0f;
uint8_t sys_cal = 0;
float currentLat = 0.0f;

// --- External Function Prototypes (Must exist in your project) ---
extern void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
extern void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
extern Adafruit_BNO055 bno;
extern bool initIMU();
extern void getIMUReadings();
extern void returnIMUReadings(float &h, float &r, float &p);
extern void getUSReadings();
extern void returnUSReadings(int &f, int &b, int &l, int &r, int &d45);
extern void GPS_init(GPSModule *g, int rxPin, int txPin);
extern void GPS_getReadings(GPSModule *g);
extern void GPS_returnReadings(GPSModule *g, double *lat, double *lon, int *sats, double *alt, double *x, double *y);
extern float calculate_left_wall_angle(AggregatedLidarFrame *frame);
void displayCommandOnOLED(const char *cmd);


// ---------------- Path Control Logic (wall_fix) ----------------

// Defined in new_path.h (or here for completeness)


// ---------------- Tasks ----------------
void printResetReason() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.printf("\n[RESET] reason=%d\n", (int)r);
  // Common values:
  // 1=POWERON_RESET, 3=SW_RESET, 5=DEEPSLEEP_RESET, 7=TG0WDT_SYS_RESET,
  // 8=TG1WDT_SYS_RESET, 9=RTCWDT_SYS_RESET, 10=INTRUSION_RESET,
  // 11=TGWDT_CPU_RESET, 12=SW_CPU_RESET, 13=RTCWDT_CPU_RESET,
  // 14=EXT_CPU_RESET, 15=RTCWDT_BROWN_OUT_RESET
}
void imuUsTask(void *pvParameters) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(20);

  while (1) {
    IMU_US_Data local = {};

    // I2C section (IMU)
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdPASS) {
      if (IMU_isCalibrating()) (void)IMU_calibrationStep();

      getIMUReadings();
      returnIMUReadings(local.heading, local.roll, local.pitch);

      uint8_t sys = 0, g = 0, a = 0, m = 0;
      bno.getCalibration(&sys, &g, &a, &m);
      local.sys_cal = sys;

      xSemaphoreGive(sensorMutex);
    }

    // US can be outside if it doesn't share I2C; if it does, keep a separate mutex for it
    getUSReadings();
    returnUSReadings(local.front, local.back, local.left, local.right, local.angle45);

    // Now write shared once (very short)
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(2)) == pdPASS) {
      currentSensorData = local;
      currentHeading = local.heading;
      sys_cal = local.sys_cal;
      xSemaphoreGive(sensorMutex);
    }

    vTaskDelayUntil(&last, period);
  }
}


void gpsTask(void *pvParameters) {
  GPS_Data gpsData = { 0 };
  double lat = 0, lon = 0, alt = 0, x = 0, y = 0;
  int sats = 0;

  GPS_init(&gps, 16, 17);
  Serial.println("GPS Initialized.");

  while (1) {
    // bounded parsing already inside GPS_getReadings() (maxBytes)
    GPS_getReadings(&gps);
    GPS_returnReadings(&gps, &lat, &lon, &sats, &alt, &x, &y);

    gpsData.lat = lat;
    gpsData.lon = lon;
    gpsData.alt = alt;
    gpsData.sats = sats;
    gpsData.x = x;
    gpsData.y = y;

    // keep latest GPS available for everyone
    (void)xQueueOverwrite(gpsDataQueue, &gpsData);

    // OLED lat (optional)
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdPASS) {
      currentLat = (float)lat;
      xSemaphoreGive(sensorMutex);
    }

    // GPS update rate (5 Hz)
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void lidarRxTask(void *pvParameters) {
  // Note: Pins 27, 33 must match wiring
  lidar.begin(230400, 27, 33);
  LidarFrame frame;

  while (1) {
    if (lidar.readPacket()) {
      memcpy(frame.points, lidar.points, sizeof(frame.points));
      // Send (non-blocking) with drop-oldest mechanism
      if (xQueueSend(lidarDataQueue, &frame, pdMS_TO_TICKS(0)) != pdPASS) {
        LidarFrame tmp;
        if (xQueueReceive(lidarDataQueue, &tmp, pdMS_TO_TICKS(0)) == pdPASS) {
          xQueueSend(lidarDataQueue, &frame, pdMS_TO_TICKS(0));
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(LD_RATE));
  }
}

void lidarProcTask(void *pvParameters) {
  LidarFrame packet;
  static int packetCount = 0;
  const int packetsPerFrame = FULL_SCAN_POINTS / LD20_Lidar::POINTS_PER_FRAME;

  while (1) {

    if (xQueueReceive(lidarDataQueue, &packet, pdMS_TO_TICKS(100)) == pdPASS) {

      /* -------------------------------------------------
             * Accumulate packets into full 360° frame
             * ------------------------------------------------- */
      for (int i = 0; i < LD20_Lidar::POINTS_PER_FRAME; ++i) {

        float angle = packet.points[i].angle;
        int index = (int)(angle + 0.5f);

        if (index >= FULL_SCAN_POINTS) index -= FULL_SCAN_POINTS;
        if (index < 0) index += FULL_SCAN_POINTS;

        fullScanFrame.points[index] = packet.points[i];
      }

      packetCount++;

      /* -------------------------------------------------
             * Full frame ready
             * ------------------------------------------------- */
      if (packetCount >= packetsPerFrame) {

        WallResult left_wall = calculate_wall_angle_and_distance(&fullScanFrame, "left");
        WallResult right_wall = calculate_wall_angle_and_distance(&fullScanFrame, "right");
        WallResult front_wall = calculate_wall_angle_and_distance(&fullScanFrame, "front");


        if (xSemaphoreTake(lidarAngleMutex, pdMS_TO_TICKS(5)) == pdPASS) {
          if (left_wall.valid) {
            currentleftWallAngle = left_wall.angleDeg;
            currentleftWallDistance = left_wall.distance_m;
          } else {
            currentleftWallAngle = 999.0f;
            currentleftWallDistance = 999.0f;
          }

          if (right_wall.valid) {
            currentrightWallAngle = right_wall.angleDeg;
            currentrightWallDistance = right_wall.distance_m;
          } else {
            currentrightWallAngle = 999.0f;
            currentrightWallDistance = 999.0f;
          }

          if (front_wall.valid) {
            currentfrontWallAngle = front_wall.angleDeg;
            currentfrontWallDistance = front_wall.distance_m;
          } else {
            currentfrontWallAngle = 999.0f;
            currentfrontWallDistance = 999.0f;
          }

          xSemaphoreGive(lidarAngleMutex);
          fullScanFrame.isComplete = true;
        }
        packetCount = 0;
      }
    }
  }
}


void dataCombineTask(void *pvParameters) {
  IMU_US_Data localSensorData = {};
  GPS_Data localGpsData = {};

  // Keep last known lidar values (so they're never uninitialized)
  float left = 999, right = 999, front = 999;
  float left_dis = 999, right_dis = 999, front_dis = 999;

  static char data[512];
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(WEBSOCKET_RATE_MS);

  for (;;) {
    // ---- IMU/US snapshot ----
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdPASS) {
      localSensorData = currentSensorData;
      xSemaphoreGive(sensorMutex);
    }

    // ---- GPS snapshot (do NOT consume queue) ----
    // Use peek so you always have latest without emptying the queue.
    (void)xQueuePeek(gpsDataQueue, &localGpsData, 0);

    // ---- Lidar snapshot ----
    if (xSemaphoreTake(lidarAngleMutex, pdMS_TO_TICKS(5)) == pdPASS) {
      left = currentleftWallAngle;
      right = currentrightWallAngle;
      front = currentfrontWallAngle;

      left_dis = currentleftWallDistance;
      right_dis = currentrightWallDistance;
      front_dis = currentfrontWallDistance;

      xSemaphoreGive(lidarAngleMutex);
    }

    // ---- Build CSV in SAME order as your working version ----
    // Note web_cmd is a String; convert safely:
    char cmd[16];
    getCmd(cmd, sizeof(cmd));

    int n = snprintf(
      data, sizeof(data),
      "%.2f,%.2f,%.2f,"  // roll,pitch,heading
      "%.6f,%.6f,%.2f,"  // lat,lon,alt
      "%d,%d,%d,%d,%d,"  // angle45,front,left,right,back
      "%d,"              // sats
      "%.2f,%.2f,"       // x,y
      "%s,"              // web_cmd
      "%.2f,%.2f,%.2f,"  // left,right,front
      "%.2f,%.2f,%.2f",  // left_dis,right_dis,front_dis
      localSensorData.roll, localSensorData.pitch, localSensorData.heading,
      localGpsData.lat, localGpsData.lon, localGpsData.alt,
      localSensorData.angle45, localSensorData.front, localSensorData.left,
      localSensorData.right, localSensorData.back,
      localGpsData.sats,
      localGpsData.x, localGpsData.y,
      cmd,
      left, right, front,
      left_dis, right_dis, front_dis);

    if (n > 0 && n < (int)sizeof(data)) {
      webSocket.broadcastTXT((const uint8_t *)data, (size_t)n);
    }

    vTaskDelayUntil(&last, period);
  }
}


static Cmd align_wall_cmd(const char *wall, float angle) {
  float wall_angle = 999.0f;

  if (xSemaphoreTake(lidarAngleMutex, pdMS_TO_TICKS(2)) != pdPASS) return CMD_STOP;
  if (!strcmp(wall, "left")) wall_angle = currentleftWallAngle;
  else if (!strcmp(wall, "right")) wall_angle = currentrightWallAngle;
  else if (!strcmp(wall, "front")) wall_angle = currentfrontWallAngle;
  xSemaphoreGive(lidarAngleMutex);

  if (wall_angle > 900.0f) return CMD_STOP;  // NO_WALL case, decide behavior

  float err = wall_angle - angle;
  if (fabsf(err) <= 20.0f) return CMD_STOP;  // aligned
  return (err > 0) ? CMD_ROT_R : CMD_ROT_L;
}


static Cmd align_IMU_cmd(float headingDeg, float targetAngle) {
  float diff = targetAngle - headingDeg;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  if (fabsf(diff) < 10) return CMD_STOP;  // or a special CMD_ALIGNED if you want
  return (diff > 0) ? CMD_ROT_R : CMD_ROT_L;
}

static String align_IMU(float headingDeg, float targetAngle) {
  return String(cmdText(align_IMU_cmd(headingDeg, targetAngle)));
}

static String align_wall(const char *wall, float angle) {
  return String(cmdText(align_wall_cmd(wall, angle)));
}


static void readPoseAndSensors(Pose2D &pose, SensorsSnap &s) {
  // IMU + US
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdPASS) {
    s.imuUs = currentSensorData;
    xSemaphoreGive(sensorMutex);
  }
  pose.heading = s.imuUs.heading;

  // GPS local x/y
  GPS_Data latest;
  if (xQueuePeek(gpsDataQueue, &latest, pdMS_TO_TICKS(10)) == pdPASS) {
    pose.x = latest.x;
    pose.y = latest.y;
  }

  // LiDAR wall distance (front)
  if (xSemaphoreTake(lidarAngleMutex, pdMS_TO_TICKS(5)) == pdPASS) {
    s.frontWallDist_m = currentfrontWallDistance;
    xSemaphoreGive(lidarAngleMutex);
  }
}

float angle_add(float a, float b) {
  float t = a + b;
  while (t >= 360) t -= 360;
  while (t < 0) t += 360;
  return t;
}

float angle_sub(float a, float b) {
  float t = a - b;  // FIXED
  while (t >= 360) t -= 360;
  while (t < 0) t += 360;
  return t;
}
static int pros = 0 ;
float distance = 999.9 ;
bool near = false ;
static String navCommandToWaypoint(const Pose2D &pose, int wpIndex) {
  double targetX, targetY;
  convertToLocal(WP_LAT[wpIndex], WP_LON[wpIndex], &targetX, &targetY);
  double dx = targetX-pose.x ;
  double dy = targetY-pose.y ;
  distance = sqrt(dx*dx + dy*dy);
  if(distance <= 6.0){
    near = true ;
  }
  else if(wpIndex == 4 && distance <= 10.0 ){
    near = true ;
  }
  else{
    near = false ;
  }
  return getPathCommand(pose.x, pose.y, targetX - x_off, targetY - y_off, pose.heading);
}

int left_right = 0; // 0 none, 1 left chosen, 2 right chosen

static void applyObstacleAvoidance(const SensorsSnap &s, String &cmd, NavState &st) {

  const int FRONT_TRIG   = 70;  // start avoidance
  const int MIN_FRONT    = 40;
  const int SIDE_SAFE    = 70;  // keep side clearance
  const int CLEAR_FRONT  = 90;  // hysteresis reset (was 70)

  // Keep your STOP logic exactly as-is
  if (s.imuUs.front <= FRONT_TRIG && near) {
    cmd = "STOP";
    return;
  }

  // Reset lock only when clearly free
  if (s.imuUs.front > CLEAR_FRONT) {
    left_right = 0;
  }

  if (s.imuUs.front <= MIN_FRONT ){
    cmd = "BACK";
    return ;
  }


  if (cmd == "FWD") {

    // 1) ALWAYS keep side distance (do NOT block with left_right)
    if (s.imuUs.right <= SIDE_SAFE) { 
      cmd = "LEFT";  
      if(s.imuUs.front <= FRONT_TRIG && left_right == 2)
      left_right = 1 ;
      return; }
    if (s.imuUs.left  <= SIDE_SAFE) {
      cmd = "RIGHT";
      if(s.imuUs.front <= FRONT_TRIG && left_right == 1)
      left_right = 2 ;
      return; }

    // 2) If front is close, choose the better side (instead of always LEFT)
    if (s.imuUs.front <= FRONT_TRIG) {

      // If we already committed to a side, keep it (anti flip-flop)
      if (left_right == 1) { cmd = "LEFT";  return; }
      if (left_right == 2) { cmd = "RIGHT"; return; }

      // Choose side with more clearance
      if (s.imuUs.left >= s.imuUs.right) {
        cmd = "LEFT";
        left_right = 1;
      } else {
        cmd = "RIGHT";
        left_right = 2;
      }
      return;
    }
  }
}




static void handleWaypointStop(NavState &st, const String &cmd) {
  if (cmd == "STOP" && st.wpIndex < NUM_WP) {
    st.wpIndex++;
    setCmd(cmd.c_str());
    displayCommandOnOLED(cmd.c_str());
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  if (cmd == "STOP" && st.wpIndex >= NUM_WP) {
    st.phase = 1;  // enter final approach
  }
}

static String finalApproachStep(NavState &st, const SensorsSnap &s) {
  // Align to heading 266.0, then move forward until front wall distance <= 2.8m
  String wall_status = align_IMU(s.imuUs.heading, 306.0);

  String cmd;
  if (wall_status == "STOP") {
    cmd = "FWD";
    st.aligned = true;
  } else {
    cmd = wall_status;
    st.aligned = false;
  }

  if ((s.imuUs.front <= 255 || s.frontWallDist_m <= 2.8f) && st.aligned) { //s.frontWallDist_m <= 2.8f || 
    st.phase = 2;
  }
  return cmd;
}

static String fineTuneWallStep(NavState &st) {
  String wall_status = align_wall("front", 90);

  String cmd;
  if (wall_status == "STOP") {
    cmd = "STOP";
    st.phase = 3;
  } 
  else {
    cmd = wall_status ;
  } 
  return cmd;
}

void pathToH(void *pvParameters) {
  NavState st = {};     // <-- critical
  Pose2D pose = {};
  SensorsSnap s = {};

  String cmd = "STOP";
  setCmd("STOP");
  displayCommandOnOLED(cmd.c_str());
  st.aligned = false;

  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(50);
  while (1) {
    if (pros != 1){
      st.phase = 3 ;
    }
    readPoseAndSensors(pose, s);
    if (st.phase == 0) {
      if (st.wpIndex < NUM_WP) { //-----------------------------------------------------------------------------------------------------------
        cmd = navCommandToWaypoint(pose, st.wpIndex);
        applyObstacleAvoidance(s, cmd, st);
        handleWaypointStop(st, cmd);
      } else {
        st.phase = 1;
      }
    }

    // -------- Phase 1: final approach --------
    if (st.phase == 1) {
      cmd = finalApproachStep(st, s);
    }

    // -------- Phase 2: fine tuning --------
    if (st.phase == 2) {
      cmd = fineTuneWallStep(st);
    }
    if (st.phase == 3) {
      cmd = "STOP";
      setCmd(cmd.c_str());
      displayCommandOnOLED(cmd.c_str());
      if(Robot_connected)
      displayCommandOnOLED("lockLaydownStand()");
      // lockLaydownStand();
      pathToHHandle = NULL;   // optional: better if protected, but usually OK here
      vTaskDelete(NULL); 
    }
    setCmd(cmd.c_str());
    displayCommandOnOLED(cmd.c_str());
    vTaskDelayUntil(&last, period);
  }
}
static String navCommandToWaypoint1(const Pose2D &pose) {
  double targetX, targetY;
  convertToLocal(WP_LAT_1, WP_LON_1, &targetX, &targetY);
  double dx = targetX-pose.x ;
  double dy = targetY-pose.y ;
  double dist = sqrt(dx*dx + dy*dy);
  if(dist <= 15){ //-------------------------------------------------------------------------------------------------------------------------------------------
    near = true ;
  }
  return getPathCommand(pose.x, pose.y, targetX - x_off, targetY - y_off, pose.heading);
}

bool verynear = false ;

static void applyObstacleAvoidance1(const SensorsSnap &s, String &cmd, NavState &st) {

  const int FRONT_TRIG   = 70;  // start avoidance
  const int MIN_FRONT    = 40;
  const int SIDE_SAFE    = 70;  // keep side clearance
  const int CLEAR_FRONT  = 90;  // hysteresis reset (was 70)

  // Keep your STOP logic exactly as-is
  if (s.imuUs.front <= FRONT_TRIG && (s.imuUs.left <= SIDE_SAFE || s.imuUs.right <= SIDE_SAFE) && near) {
    verynear = true ;
    return;
  }

  // Reset lock only when clearly free
  if (s.imuUs.front > CLEAR_FRONT) {
    left_right = 0;
  }

  if (s.imuUs.front <= MIN_FRONT){
    cmd = "BACK";
    return ;
  }

  if (cmd == "FWD") {

    // 1) ALWAYS keep side distance (do NOT block with left_right)
    if (s.imuUs.right <= SIDE_SAFE) { 
      cmd = "LEFT";  
      if(s.imuUs.front <= FRONT_TRIG && left_right == 2)
      left_right = 1 ;
      return; }
    if (s.imuUs.left  <= SIDE_SAFE) {
      cmd = "RIGHT";
      if(s.imuUs.front <= FRONT_TRIG && left_right == 1)
      left_right = 2 ;
      return; }

    // 2) If front is close, choose the better side (instead of always LEFT)
    if (s.imuUs.front <= FRONT_TRIG) {
      if (near){
        left_right = 1 ;
      }

      // If we already committed to a side, keep it (anti flip-flop)
      if (left_right == 1) { cmd = "LEFT";  return; }
      if (left_right == 2) { cmd = "RIGHT"; return; }

      // Choose side with more clearance
      if (s.imuUs.left >= s.imuUs.right) {
        cmd = "LEFT";
        left_right = 1;
      } else {
        cmd = "RIGHT";
        left_right = 2;
      }
      return;
    }
  }
}

bool opst = false ;

static String finalApproachStep1(NavState &st, SensorsSnap &s , Pose2D &pose) {
  // Align to heading 266.0, then move forward until front wall distance <= 2.8m
  String wall_status = align_IMU(s.imuUs.heading, 37.8);

  String cmd;
  if (wall_status == "STOP") {
    readPoseAndSensors(pose, s);
    st.aligned = true;
  } else {
    cmd = wall_status;
    st.aligned = false;
  }

  if (s.imuUs.front >= 130 && st.aligned) {// s.frontWallDist_m >= 1.55f
    cmd = "FWD";
  }
  else if (st.aligned){
    cmd = "STOP" ;
    verynear = true ;
  }
  return cmd;
}

void pathToPost(void *pvParameters) {
  NavState st = {};     // <-- critical
  Pose2D pose = {};
  SensorsSnap s = {};
  verynear = false ;
  near = false ;

  String cmd = "STOP";
  setCmd("STOP");
  displayCommandOnOLED(cmd.c_str());

  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(50);
  
  while (1) {
    if (pros != 2){
      verynear = true ;
    }
        readPoseAndSensors(pose, s);
        cmd = navCommandToWaypoint1(pose);
        applyObstacleAvoidance1(s, cmd, st); 
        //if(near){
        //cmd = finalApproachStep1(st, s , pose); 
        //}   
      if(verynear){
      cmd = "STOP" ;
      setCmd(cmd.c_str());
      displayCommandOnOLED(cmd.c_str());
      pathToPostHandle = NULL;
      vTaskDelete(NULL);
    }
    setCmd(cmd.c_str());
    displayCommandOnOLED(cmd.c_str());
    vTaskDelayUntil(&last, period);
  }
}

static inline void suspendTaskSafe(TaskHandle_t h, TaskHandle_t current) {
  if (h && h != current) vTaskSuspend(h);
}

static inline void resumeTaskSafe(TaskHandle_t h) {
  if (h) vTaskResume(h);
}
void suspendAllRobotTasks() {
  TaskHandle_t current = xTaskGetCurrentTaskHandle();

  suspendTaskSafe(hRobotControl, current);
  suspendTaskSafe(hWebServer, current);
  suspendTaskSafe(hDataCombine, current);
  suspendTaskSafe(hIMU_US, current);
  suspendTaskSafe(hGPS, current);
  suspendTaskSafe(hLidarRx, current);
  suspendTaskSafe(hLidarProc, current);
  suspendTaskSafe(hOLED_Update, current);
}
void resumeAllRobotTasks() {
  // Resume everything (order not super critical)
  resumeTaskSafe(hRobotControl);
  resumeTaskSafe(hWebServer);
  resumeTaskSafe(hDataCombine);
  resumeTaskSafe(hIMU_US);
  resumeTaskSafe(hGPS);
  resumeTaskSafe(hLidarRx);
  resumeTaskSafe(hLidarProc);
  resumeTaskSafe(hOLED_Update);
}

void startNavigationToHBuilding() {
  pros = 1 ;

  near = false;
  verynear = false;
  opst = false;

  if (!pathToHHandle) {
    xTaskCreatePinnedToCore(pathToH, "pathToH", 8192, NULL, 6, &pathToHHandle, 1);
  }
}

void startNavigationToPostOffice() {
  pros = 2 ;

  near = false;
  verynear = false;
  opst = false;

  if (!pathToPostHandle) {
    xTaskCreatePinnedToCore(pathToPost, "pathToPost", 8192, NULL, 6, &pathToPostHandle, 1);
  }
}

void stopAllMovement() {
  pros = 3 ;

  if(Robot_connected)
  displayCommandOnOLED("stop()");
  // stop();

  near = false;
  verynear = false;
  opst = false;
}

void init_Check() {
  if(Robot_connected){
    suspendAllRobotTasks();
    displayCommandOnOLED("initialcheckuproutine()");
    // initialcheckuproutine();
    resumeAllRobotTasks();
  }
  else{
    Serial.println("Failed to do Initial Checkup");
  }
}

void webServerTask(void *pvParameters) {

  WiFi.mode(WIFI_AP);
  WiFi.softAP("SWAS2026", "12345678");
  delay(200);

  // ... (unchanged ESP-NOW setup) ...
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
  }

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  server.on("/", []() {
    server.send_P(200, "text/html", webpage);
  });
  server.begin();

  while (1) {
    webSocket.loop();
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void oledUpdateTask(void *pvParameters) {
  while (1) {
    // Only need to ensure IMU globals are updated before use
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdPASS) {
      // Now, currentHeading and sys_cal are fresh copies
      xSemaphoreGive(sensorMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void robo_control(void *pvParameters){
  
  char cmd[16];
  while(1){
    if (checkupRequested) {
      checkupRequested = false;
      init_Check();
    }
    getCmd(cmd, sizeof(cmd));
    if((strcmp(cmd,"FWD")==0)){
      displayCommandOnOLED("forward(28)");
      // forward(28);
    }
    if((strcmp(cmd,"LEFT")==0)){
      displayCommandOnOLED("stepLeft(28)");
      // stepLeft(28);
    }
    if((strcmp(cmd,"RIGHT")==0)){
      displayCommandOnOLED("stepRight(28)");
      // stepRight(28);
    }
    if((strcmp(cmd,"ROT_L")==0)){
      displayCommandOnOLED("rotateLeft(28)");
      // rotateLeft(28);
    }
    if((strcmp(cmd,"ROT_R")==0)){
      displayCommandOnOLED("rotateRight(28)");
      // rotateRight(28);
    }
    if((strcmp(cmd,"STOP")==0)){
      displayCommandOnOLED("stop()");
      // stop();
    }
    if((strcmp(cmd,"BACK")==0)){
      displayCommandOnOLED("backward(28)");
      // backward(28);
    }
    vTaskDelay(pdMS_TO_TICKS(90)); // -------------------------------------------------------------------------------------------------------------
  }
}


void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(500); // Give hardware time to stabilize
  Wire.begin(); 
  Wire.setClock(400000); // Optional: Set I2C to 400kHz for faster updates
  
  printResetReason();

  // 1. Create Sync Objects (CRITICAL: Do this before any hardware init)
  sensorMutex     = xSemaphoreCreateMutex();
  lidarAngleMutex = xSemaphoreCreateMutex();
  cmdMutex        = xSemaphoreCreateMutex();
  lidarDataQueue  = xQueueCreate(LIDAR_QUEUE_SIZE, sizeof(LidarFrame));
  gpsDataQueue    = xQueueCreate(GPS_QUEUE_SIZE, sizeof(GPS_Data));

  if (!sensorMutex || !lidarAngleMutex || !lidarDataQueue || !gpsDataQueue || !cmdMutex) {
    Serial.println("FATAL: FreeRTOS object creation failed!");
    while (1) delay(1000);
  }

  // 2. Initialize Hardware Peripherals (Single-threaded mode)
  Serial.println("Initializing Peripherals...");
  
  if (!initOLED())   Serial.println("OLED Init Failed");
  if (!initIMU())    Serial.println("IMU Init Failed");
  //if (!initExpMod()) Serial.println("Expander Init Failed");
  //if (!initDAC())    Serial.println("DAC Init Failed");

  // 3. Blocking Calibration (Runs before multi-tasking starts)
  if (!imuOffsetsLoaded) {
  Serial.println("Starting IMU Calibration (Blocking)...");
    //IMU_startCalibration(240000); //---------------------------------------------------------------------------------------------
  } else {
    Serial.println("[IMU] EEPROM offsets loaded.");
  }

  // 4. Initial Physical Checkup
  // Note: expMod and DAC are no longer in the electrical circuit
  // Serial.println("Running Robot Checkup Routine...");
  // if (!initExpMod() || !initDAC()) {
  // Serial.println("----------------FAILED---------------ROBOT NOT CONNECTED");
  // }
  // else{
  Robot_connected = true ;
  Serial.println("System ready. Launching Tasks...");
  xTaskCreatePinnedToCore(robo_control,   "RobotControl", 4096, NULL, 2, &hRobotControl, 0);
  // }
  // 5. Create Tasks (Launch the engine)

  // Core 0 Tasks
  xTaskCreatePinnedToCore(webServerTask,   "WebServer",   8192, NULL, 1, &hWebServer,   0);
  xTaskCreatePinnedToCore(dataCombineTask, "DataCombine", 4096, NULL, 1, &hDataCombine, 0);

  // Core 1 Tasks
  xTaskCreatePinnedToCore(imuUsTask,      "IMU_US",      4096, NULL, 5, &hIMU_US,      0);
  xTaskCreatePinnedToCore(gpsTask,        "GPS",         4096, NULL, 3, &hGPS,         1);
  xTaskCreatePinnedToCore(lidarRxTask,    "LiDAR_Rx",    4096, NULL, 4, &hLidarRx,     1);
  xTaskCreatePinnedToCore(lidarProcTask,  "LidarProc",   4096, NULL, 4, &hLidarProc,   1);
  xTaskCreatePinnedToCore(oledUpdateTask, "OLED_Update", 2048, NULL, 1, &hOLED_Update, 1);

  Serial.println("All tasks running.");
}

void loop() {
  vTaskDelete(NULL);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  (void)num;

  switch (type) {
    case WStype_DISCONNECTED:
      break;

    case WStype_CONNECTED:
      break;

    case WStype_TEXT:
      {
        char cmdIn[32];
        size_t n = (length < sizeof(cmdIn) - 1) ? length : sizeof(cmdIn) - 1;
        memcpy(cmdIn, payload, n);
        cmdIn[n] = '\0';

        if (strcmp(cmdIn, "H BUILDING") == 0) startNavigationToHBuilding();
        else if (strcmp(cmdIn, "POST BUILDING") == 0) startNavigationToPostOffice();
        else if (strcmp(cmdIn, "STOP") == 0) stopAllMovement();
        else if (strcmp(cmdIn, "CHECKUP") == 0) checkupRequested = true;
        break;
      }

    default:
      break;
  }
}
