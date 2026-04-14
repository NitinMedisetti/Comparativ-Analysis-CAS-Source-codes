#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WMM_Tinier.h>
#include <array>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// Project Headers
#include "Ultrasonic.h"
#include "GPS.h"
#include "IMU.h"
#include "Display.h"    
#include "SensorData.h" 
#include "Geolocation.h"
#include "Navigation.h" 
// #include "Controller.h"  // Disabled - running in simulation mode 

// ==================================================================================
// CONFIGURATION
// ==================================================================================
#define DEV_LED_PIN 2
#define MAGNETIC_DECLINATION 4.2f
#define YAW_NORTH_OFFSET 100.0f

// --- BLIND MODE 1 CONFIGURATION (START) ---
#define HEADING_BM1_E   125.0f   // Target Heading 125 degrees
#define DIST_BM1_LEG_E  16.0f    // Forward Distance 16 meters

// --- NEW BLIND MODE (MIDDLE) CONFIGURATION ---
// After Point 2: Align 200 degrees, go 32 meters
#define HEADING_MID_BLIND 200.0f
#define DIST_MID_BLIND    32.0f

// --- BLIND MODE 2 CONFIGURATION (END) ---
#define HEADING_BM2_W   210.0f   
#define DIST_BM2_LEG    10.0f    

// --- OBSTACLE AVOIDANCE CONFIGURATION ---
#define OBSTACLE_DIST_THRESH 60   
#define OBSTACLE_EXIT_THRESH  110  
#define SIDE_OBSTACLE_THRESH  40  
#define RAD_TO_DEG 57.2957795131 

// WMM DATE
const uint8_t WMM_YEAR = 25;
const uint8_t WMM_MONTH = 1;
const uint8_t WMM_DAY = 1;

// ==================================================================================
// WAYPOINT MAPS
// ==================================================================================
struct WayPoints {
    static constexpr double MONUMENT_LAT = 48.829950;
    static constexpr double MONUMENT_LON = 12.955072;
    static constexpr double POINT1_LAT = 48.830067;  // Index 0 (Skipped)
    static constexpr double POINT1_LON = 12.954973;  
    static constexpr double POINT2_LAT = 48.829765;  // Index 1 (Stop here -> Trigger New Blind Mode)
    static constexpr double POINT2_LON = 12.954965;
    // Point 3 Removed as requested
    static constexpr double POINT4_LAT = 48.829572;  // Index 2 (Resume here after New Blind Mode)
    static constexpr double POINT4_LON = 12.954370;  
};

// ==== PATH DEFINITION ====
struct GeoPoint { double lat; double lon; };

GeoPoint gpsPath[] = {
    {WayPoints::POINT1_LAT, WayPoints::POINT1_LON}, // Index 0 (Skipped)
    {WayPoints::POINT2_LAT, WayPoints::POINT2_LON}, // Index 1 (Target after BM1)
    {WayPoints::POINT4_LAT, WayPoints::POINT4_LON}, // Index 2 (Target after Mid Blind)
};
const int TOTAL_WAYPOINTS = sizeof(gpsPath) / sizeof(gpsPath[0]);

const int IDX_PT1 = 0;
const int IDX_PT2 = 1; 
const int IDX_PT4 = 2; // Was Point 4, now index 2 in array

struct LocalPoint { float n; float e; };
LocalPoint localPath[TOTAL_WAYPOINTS];

// ==================================================================================
// FREE RTOS & SYNC OBJECTS
// ==================================================================================
TaskHandle_t Task0;
TaskHandle_t InitTaskHandle = NULL; 

// Mutexes
SemaphoreHandle_t dataMutex; // Protects sharedData struct
SemaphoreHandle_t i2cMutex;  // Protects I2C Bus

// Hardware Flag
bool motorHardwareAvailable = false; 

struct SharedData {
    // Inputs
    uint16_t us_front, us_left, us_right, us_back, us_incline;
    float yaw;
    float gyro_z;
    double lat, lon;
    bool gps_valid;
    
    // Outputs
    SystemState state;
    NavCommand cmd;
    float dist_to_target;
    int current_wp_index;
    bool is_mission_active;
} sharedData;

// ==================================================================================
// MISSION STATE MACHINE
// ==================================================================================
enum MissionStep {
    STEP_IDLE,
    STEP_NAV_TO_POINT1,      // DEPRECATED / SKIPPED
    STEP_BM1_ALIGN_EAST,     // Align 125
    STEP_BM1_FWD_LEG,        // Forward 16m
    STEP_BM1_PAUSED,         
    STEP_GPS_NAVIGATION,     // Generic GPS Navigation State
    STEP_MID_BLIND_ALIGN,    // NEW: Align 200 deg after Pt2
    STEP_MID_BLIND_FWD,      // NEW: Forward 32m
    STEP_BM2_ALIGN_WEST,     // Existing End Blind Mode
    STEP_BM2_FWD_TO_PT6,    
    STEP_RETURN_NAV, 
    STEP_DONE
};

enum MissionType {
    MISSION_FULL,
    MISSION_POST_ONLY,
    MISSION_RETURN 
};

MissionStep currentMissionStep = STEP_IDLE;
MissionType currentMissionType = MISSION_FULL; 

double legStartLat = 0.0;
double legStartLon = 0.0;
float distTraveledLeg = 0.0; 

// ==================================================================================
// GLOBAL OBJECTS
// ==================================================================================
UltrasonicSensor usFront(32, 25);
UltrasonicSensor usBack(5, 14);
UltrasonicSensor usLeft(4, 18);
UltrasonicSensor usRight(13, 19);
UltrasonicSensor usInclined(23, 26);
GPSModule gps(Serial2);
IMU imu2(0x28);
Display display;
WMM_Tinier wmm;
Geolocation geo(0.0, 0.0);
Navigation navigator;

struct_message espNowData;
CommandMessage receivedCmd;

// --- GLOBAL FLAGS FOR STOP/RESUME ---
volatile bool emergencyStopTriggered = false;
bool isMissionPaused = false; 
volatile bool triggerInitRoutine = false; 
volatile bool isInitializing = false; 
// ----------------------------------------

float magneticDeclination = MAGNETIC_DECLINATION;
const float GPS_FILTER_ALPHA = 0.5f; 
double filteredLat = 0.0, filteredLon = 0.0;
bool isFirstGpsReading = true;

// Timings
unsigned long lastBasicSendTime = 0;
unsigned long lastDisplayTime = 0;
const long BASIC_SEND_INTERVAL = 200;
const long DISPLAY_INTERVAL = 100;

uint8_t espCamAddress[] = {0xD8, 0x13, 0x2A, 0xC2, 0x2E, 0x2C}; //{0xFC, 0xB4, 0x67, 0xF0, 0x8A, 0x28}  {0xD8, 0x13, 0x2A, 0xC2, 0x2E, 0x2C}

SystemState currentState = STATE_INITIALIZING;
NavCommand currentNavCommand = NAV_WAIT;

// ==================================================================================
// PROTOTYPES
// ==================================================================================
void Task0Code(void * pvParameters);
void InitTaskCode(void * pvParameters);
void sendBasicSensorData();
void convertPathToLocal();
void OnDataSent(const wifi_tx_info_t *mac, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len);
void logSensorData(); 
uint16_t normalizeUS(int val) { return (val < 0 || val > 65000) ? 999 : val; }

// ==================================================================================
// SETUP
// ==================================================================================
void setup() {
    Serial.begin(115200);
    
    // 1. Create Mutexes
    dataMutex = xSemaphoreCreateMutex();
    i2cMutex = xSemaphoreCreateMutex(); 

    // 2. Init I2C
    Wire.begin(); 
    Wire.setClock(400000); 
    Wire.setTimeOut(100); 

    // 3. Hardware Auto-Detection
    Serial.print("Checking for Motor Controller Hardware...");
    // bool dacFound = initDAC();
    // bool expFound = initExpMod();
    // Controller.h has been removed - running in SIMULATION MODE
    motorHardwareAvailable = false;
    Serial.println(" [DISABLED - Controller.h not included]");
    Serial.println("Running in SIMULATION MODE (Display-only commands)");
    
    pinMode(DEV_LED_PIN, OUTPUT);
    if (!display.begin()) Serial.println("SSD1306 failed");

    usFront.begin(); usBack.begin(); usLeft.begin(); usRight.begin(); usInclined.begin();
    if (!imu2.begin()) { Serial.println("BNO055 Failed"); while(1); }
    wmm.begin();
    gps.begin(16, 17);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); 
    
    if (esp_now_init() != ESP_OK) return;
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, espCamAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    convertPathToLocal();
    sharedData.state = STATE_WAITING_FOR_GPS;
    sharedData.cmd = NAV_WAIT;
    sharedData.is_mission_active = false;
    sharedData.current_wp_index = 0;

    xTaskCreatePinnedToCore(Task0Code, "LogicTask", 10000, NULL, 1, &Task0, 0);
    Serial.println("Dual Core System Ready.");
}

// ==================================================================================
// MAIN LOOP (CORE 1) - FAST SENSORS & MOTORS
// ==================================================================================
void loop() {
    unsigned long now = millis();
    gps.update();

    static unsigned long lastGyroReadTime = 0;
    if (now - lastGyroReadTime >= 20) { 
        lastGyroReadTime = now;
        
        float liveGyroZ = 0.0f;
        if (xSemaphoreTake(i2cMutex, (TickType_t) 5) == pdTRUE) {
            liveGyroZ = imu2.readGyroRateZ();
            xSemaphoreGive(i2cMutex);
            
            if (xSemaphoreTake(dataMutex, (TickType_t) 5) == pdTRUE) {
                sharedData.gyro_z = liveGyroZ;
                xSemaphoreGive(dataMutex);
            }
        }
    }

    if (triggerInitRoutine && !isInitializing) {
        triggerInitRoutine = false;
        xTaskCreate(InitTaskCode, "InitTask", 4096, NULL, 1, &InitTaskHandle);
    }
    
    if (isInitializing) {
        // Init routine handles its own movement
    }
    else if (emergencyStopTriggered || isMissionPaused || !sharedData.is_mission_active) {
        if (motorHardwareAvailable) {
            if (xSemaphoreTake(i2cMutex, (TickType_t) 50) == pdTRUE) {
                // stop(); 
                display.m_display.clearDisplay();
                display.m_display.setTextSize(2);
                display.m_display.setCursor(0, 20);
                display.m_display.println("stop()");
                display.m_display.display();
                xSemaphoreGive(i2cMutex);
            }
        }
    }
    else {
        NavCommand moveCmd = NAV_WAIT;
        if (xSemaphoreTake(dataMutex, (TickType_t) 5) == pdTRUE) {
            moveCmd = sharedData.cmd;
            xSemaphoreGive(dataMutex);
        }

        if (motorHardwareAvailable) {
            if (xSemaphoreTake(i2cMutex, (TickType_t) 50) == pdTRUE) {
                switch (moveCmd) {
                    case NAV_FORWARD:
                        // forward(28);
                        display.m_display.clearDisplay();
                        display.m_display.setTextSize(2);
                        display.m_display.setCursor(0, 20);
                        display.m_display.println("forward(28)");
                        display.m_display.display();
                        break; 
                    case NAV_TURN_LEFT:
                        // rotateLeft(28);
                        display.m_display.clearDisplay();
                        display.m_display.setTextSize(2);
                        display.m_display.setCursor(0, 20);
                        display.m_display.println("rotateLeft(28)");
                        display.m_display.display();
                        break; 
                    case NAV_TURN_RIGHT:
                        // rotateRight(28);
                        display.m_display.clearDisplay();
                        display.m_display.setTextSize(2);
                        display.m_display.setCursor(0, 20);
                        display.m_display.println("rotateRight(28)");
                        display.m_display.display();
                        break;
                    case NAV_STRAFE_LEFT:
                        // stepLeft(28);
                        display.m_display.clearDisplay();
                        display.m_display.setTextSize(2);
                        display.m_display.setCursor(0, 20);
                        display.m_display.println("stepLeft(28)");
                        display.m_display.display();
                        break; 
                    case NAV_STRAFE_RIGHT:
                        // stepRight(28);
                        display.m_display.clearDisplay();
                        display.m_display.setTextSize(2);
                        display.m_display.setCursor(0, 20);
                        display.m_display.println("stepRight(28)");
                        display.m_display.display();
                        break;
                    case NAV_WAIT:
                        // stop();
                        display.m_display.clearDisplay();
                        display.m_display.setTextSize(2);
                        display.m_display.setCursor(0, 20);
                        display.m_display.println("stop()");
                        display.m_display.display();
                        break;
                    default:
                        // stop();
                        display.m_display.clearDisplay();
                        display.m_display.setTextSize(2);
                        display.m_display.setCursor(0, 20);
                        display.m_display.println("stop()");
                        display.m_display.display();
                        break;
                }
                xSemaphoreGive(i2cMutex);
            }
        }
    }

    if (now - lastBasicSendTime >= BASIC_SEND_INTERVAL) {
        sendBasicSensorData();
    }
}

// ==================================================================================
// INIT TASK
// ==================================================================================
void InitTaskCode(void * pvParameters) {
    isInitializing = true;
    Serial.println(">>> EXECUTING INITIALIZATION ROUTINE...");
    
    if (motorHardwareAvailable) {
        // initialcheckuproutine(); 
        display.m_display.clearDisplay();
        display.m_display.setTextSize(2);
        display.m_display.setCursor(0, 20);
        display.m_display.println("initialcheckup");
        display.m_display.setCursor(0, 45);
        display.m_display.println("routine()");
        display.m_display.display();
        delay(2000);
    } else {
        Serial.println("Simulation Mode: Skipping physical movement routine.");
        delay(2000); 
    }
    
    isInitializing = false;
    Serial.println(">>> INITIALIZATION COMPLETE.");
    vTaskDelete(NULL); 
}

// ==================================================================================
// CORE 0 TASK - MISSION LOGIC & DISPLAY
// ==================================================================================
void Task0Code(void * pvParameters) {
    
    static float virtualYaw = 0.0f;
    static unsigned long lastIntegrationTime = 0;
    static bool wasMoving = false;
    static bool isAvoidLocked = false; 
    static NavCommand lockedAvoidDirection = NAV_WAIT; 

    for(;;) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        NavCommand computedCmd = NAV_WAIT;
        SharedData localCopy;
        
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            localCopy = sharedData;
            xSemaphoreGive(dataMutex);
        }

        // --- GYRO LOGIC ---
        unsigned long now = millis();
        float dt = (now - lastIntegrationTime) / 1000.0f; 
        lastIntegrationTime = now;

        bool isRobotMoving = (localCopy.cmd != NAV_WAIT && localCopy.cmd != NAV_STOPPED);

        if (!isRobotMoving) {
            virtualYaw = localCopy.yaw;
            wasMoving = false;
        } 
        else {
            virtualYaw -= (localCopy.gyro_z * dt); 
            if (virtualYaw < 0.0f) virtualYaw += 360.0f;
            if (virtualYaw >= 360.0f) virtualYaw -= 360.0f;
            wasMoving = true;
        }
        
        float currentYaw = virtualYaw; 

        if (localCopy.is_mission_active && !isInitializing) {
            
            if (isMissionPaused) {
                computedCmd = NAV_STOPPED; 
                isAvoidLocked = false; 
                wasMoving = false;
            } 
            else { 
                NavCommand desiredNavCmd = NAV_WAIT;

                // Calculate distances for GPS legs
                if (currentMissionStep == STEP_NAV_TO_POINT1 || 
                    currentMissionStep == STEP_GPS_NAVIGATION ||
                    currentMissionStep == STEP_RETURN_NAV) {
                    
                    int idx = localCopy.current_wp_index;
                    if(idx < 0) idx = 0;
                    if(idx >= TOTAL_WAYPOINTS) idx = TOTAL_WAYPOINTS - 1;

                    float tN = localPath[idx].n;
                    float tE = localPath[idx].e;
                    float cN, cE;
                    geo.toLocalCoordinates(localCopy.lat, localCopy.lon, cN, cE);
                    distTraveledLeg = geo.getLocalDistance(cN, cE, tN, tE);
                    
                    desiredNavCmd = navigator.computeLocalPathCommand(cN, cE, tN, tE, currentYaw);
                } 
                else if (legStartLat != 0.0 && localCopy.gps_valid) {
                    // For blind legs, calculate distance from start of leg
                    distTraveledLeg = geo.haversineDistance(legStartLat, legStartLon, localCopy.lat, localCopy.lon);
                }

                // --- STATE MACHINE ---
                switch(currentMissionStep) {
                    case STEP_NAV_TO_POINT1:
                    {
                        currentMissionStep = STEP_BM1_ALIGN_EAST;
                        break;
                    }
                    case STEP_BM1_ALIGN_EAST:
                        desiredNavCmd = navigator.computeHeadingCommand(HEADING_BM1_E, currentYaw, false);
                        if (desiredNavCmd == NAV_WAIT) {
                            legStartLat = localCopy.lat; 
                            legStartLon = localCopy.lon;
                            
                            if (currentMissionType == MISSION_POST_ONLY) {
                                currentMissionStep = STEP_BM1_PAUSED;
                            } else {
                                currentMissionStep = STEP_BM1_FWD_LEG;
                            }
                        }
                        break;
                        
                    case STEP_BM1_PAUSED:
                        desiredNavCmd = NAV_WAIT;
                        if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                            sharedData.state = STATE_READY;
                            sharedData.is_mission_active = false; 
                            xSemaphoreGive(dataMutex);
                        }
                        break;
                        
                    case STEP_BM1_FWD_LEG:
                        desiredNavCmd = navigator.computeHeadingCommand(HEADING_BM1_E, localCopy.yaw, true);
                        if (distTraveledLeg >= DIST_BM1_LEG_E) {
                            // First Blind leg done. Now Navigate to Point 2.
                            currentMissionStep = STEP_GPS_NAVIGATION;
                            if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                                sharedData.current_wp_index = IDX_PT2; 
                                xSemaphoreGive(dataMutex);
                            }
                        }
                        break;
                        
                    case STEP_GPS_NAVIGATION:
                    {
                        int idx = localCopy.current_wp_index;
                        
                        // Check if we reached the current target
                        if (distTraveledLeg < 3.0f) { 
                            if (idx == IDX_PT2) {
                                // Reached Point 2 -> Start NEW Mid-Blind Mode (200 deg, 32m)
                                legStartLat = localCopy.lat; 
                                legStartLon = localCopy.lon;
                                distTraveledLeg = 0;
                                currentMissionStep = STEP_MID_BLIND_ALIGN;
                            }
                            else if (idx == IDX_PT4) {
                                // Reached Point 4 -> Start Final Blind Mode (210 deg, 10m)
                                legStartLat = localCopy.lat; 
                                legStartLon = localCopy.lon;
                                distTraveledLeg = 0;
                                currentMissionStep = STEP_BM2_ALIGN_WEST;
                            }
                            else if (idx < IDX_PT4) {
                                // Just incase there are other points, next point
                                if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                                    sharedData.current_wp_index++;
                                    xSemaphoreGive(dataMutex);
                                }
                            }
                        }
                        break;
                    }

                    // --- NEW BLIND MODE LOGIC ---
                    case STEP_MID_BLIND_ALIGN:
                        desiredNavCmd = navigator.computeHeadingCommand(HEADING_MID_BLIND, localCopy.yaw, false);
                        if (desiredNavCmd == NAV_WAIT) {
                            legStartLat = localCopy.lat; 
                            legStartLon = localCopy.lon;
                            currentMissionStep = STEP_MID_BLIND_FWD;
                        }
                        break;

                    case STEP_MID_BLIND_FWD:
                        desiredNavCmd = navigator.computeHeadingCommand(HEADING_MID_BLIND, localCopy.yaw, true);
                        if (distTraveledLeg >= DIST_MID_BLIND) {
                            // 32 meters done. Resume GPS to Point 4.
                            currentMissionStep = STEP_GPS_NAVIGATION;
                            if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                                sharedData.current_wp_index = IDX_PT4; 
                                xSemaphoreGive(dataMutex);
                            }
                        }
                        break;
                    // -----------------------------

                    case STEP_BM2_ALIGN_WEST:
                        desiredNavCmd = navigator.computeHeadingCommand(HEADING_BM2_W, localCopy.yaw, false);
                        if (desiredNavCmd == NAV_WAIT) {
                            legStartLat = localCopy.lat; 
                            legStartLon = localCopy.lon;
                            currentMissionStep = STEP_BM2_FWD_TO_PT6;
                        }
                        break;

                    case STEP_BM2_FWD_TO_PT6:
                        desiredNavCmd = navigator.computeHeadingCommand(HEADING_BM2_W, localCopy.yaw, true);
                        if (distTraveledLeg >= DIST_BM2_LEG) {
                            desiredNavCmd = NAV_WAIT;
                            currentMissionStep = STEP_DONE;
                            if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                                sharedData.state = STATE_REACHED;
                                xSemaphoreGive(dataMutex);
                            }
                        }
                        break;

                    case STEP_RETURN_NAV:
                    {
                        int idx = localCopy.current_wp_index;
                        if (distTraveledLeg < 4.0f) {
                            if (idx == 0) {
                                desiredNavCmd = NAV_WAIT;
                                currentMissionStep = STEP_DONE;
                                if (xSemaphoreTake(dataMutex, (TickType_t)10)) {
                                    sharedData.state = STATE_REACHED; 
                                    sharedData.is_mission_active = false;
                                    xSemaphoreGive(dataMutex);
                                }
                            } else {
                                if (xSemaphoreTake(dataMutex, (TickType_t)10)) {
                                    sharedData.current_wp_index--; 
                                    xSemaphoreGive(dataMutex);
                                }
                            }
                        }
                        break;
                    }
                    case STEP_DONE:
                        desiredNavCmd = NAV_WAIT;
                        break;
                    default: 
                        desiredNavCmd = NAV_WAIT; break;
                }

                computedCmd = desiredNavCmd; 

                // --- OBSTACLE AVOIDANCE ---
                int targetIdx = localCopy.current_wp_index;
                if(targetIdx < 0) targetIdx = 0; 
                if(targetIdx >= TOTAL_WAYPOINTS) targetIdx = TOTAL_WAYPOINTS - 1;

                float tN = localPath[targetIdx].n;
                float tE = localPath[targetIdx].e;
                float cN, cE;
                geo.toLocalCoordinates(localCopy.lat, localCopy.lon, cN, cE);

                float dN = tN - cN;
                float dE = tE - cE;
                float targetHeading = atan2(dE, dN) * RAD_TO_DEG;
                if (targetHeading < 0) targetHeading += 360.0f;
                
                float headingDiff = targetHeading - localCopy.yaw;
                while (headingDiff < -180) headingDiff += 360; 
                while (headingDiff > 180) headingDiff -= 360;

                if (currentMissionStep != STEP_DONE && 
                    currentMissionStep != STEP_BM1_PAUSED) {
                    
                    static NavCommand latchedMove = NAV_WAIT; 
                    static unsigned long sideClearanceStartTime = 0; 
                    static unsigned long forwardBypassStartTime = 0;
                    static uint8_t frontTriggerCount = 0; 
                    bool frontBlocked = false;

                    if (latchedMove == NAV_WAIT && sideClearanceStartTime == 0 && forwardBypassStartTime == 0) {
                        if (localCopy.us_front > 0 && localCopy.us_front < OBSTACLE_DIST_THRESH) {
                            frontTriggerCount++;
                        } else {
                            frontTriggerCount = 0; 
                        }
                        if (frontTriggerCount >= 3) {
                            frontBlocked = true;
                            frontTriggerCount = 3; 
                        }
                    } 
                    else {
                        if (localCopy.us_front > 0 && localCopy.us_front < (OBSTACLE_DIST_THRESH + 20)) {
                            frontBlocked = true;
                        }
                        frontTriggerCount = 0; 
                    }

                    if (frontBlocked) {
                        if (latchedMove == NAV_STRAFE_LEFT) {
                            if (localCopy.us_left > 0 && localCopy.us_left < SIDE_OBSTACLE_THRESH) {
                                latchedMove = NAV_WAIT; 
                            }
                        }
                        else if (latchedMove == NAV_STRAFE_RIGHT) {
                            if (localCopy.us_right > 0 && localCopy.us_right < SIDE_OBSTACLE_THRESH) {
                                latchedMove = NAV_WAIT; 
                            }
                        }

                        if (latchedMove == NAV_WAIT) {
                            bool leftIsSafe = (localCopy.us_left == 0 || localCopy.us_left > SIDE_OBSTACLE_THRESH);
                            bool rightIsSafe = (localCopy.us_right == 0 || localCopy.us_right > SIDE_OBSTACLE_THRESH);

                            if (leftIsSafe && rightIsSafe) {
                                if (headingDiff < -10.0f) latchedMove = NAV_STRAFE_LEFT;
                                else if (headingDiff > 10.0f) latchedMove = NAV_STRAFE_RIGHT;
                                else {
                                     if (localCopy.us_left > localCopy.us_right) latchedMove = NAV_STRAFE_LEFT;
                                     else latchedMove = NAV_STRAFE_RIGHT;
                                }
                            } 
                            else if (leftIsSafe) latchedMove = NAV_STRAFE_LEFT;
                            else if (rightIsSafe) latchedMove = NAV_STRAFE_RIGHT;
                            else latchedMove = NAV_WAIT; 
                        }
                        
                        computedCmd = latchedMove;
                        sideClearanceStartTime = 0;
                        forwardBypassStartTime = 0;
                    } 
                    else if (latchedMove != NAV_WAIT) {
                        if (sideClearanceStartTime == 0) sideClearanceStartTime = millis();
                        
                        if (millis() - sideClearanceStartTime < 3000) {
                            if (latchedMove == NAV_STRAFE_LEFT && localCopy.us_left > 0 && localCopy.us_left < SIDE_OBSTACLE_THRESH) {
                                computedCmd = NAV_WAIT; latchedMove = NAV_WAIT; 
                            }
                            else if (latchedMove == NAV_STRAFE_RIGHT && localCopy.us_right > 0 && localCopy.us_right < SIDE_OBSTACLE_THRESH) {
                                computedCmd = NAV_WAIT; latchedMove = NAV_WAIT; 
                            }
                            else computedCmd = latchedMove; 
                        } 
                        else {
                            if (forwardBypassStartTime == 0) forwardBypassStartTime = millis();
                            if (millis() - forwardBypassStartTime < 5000) computedCmd = NAV_FORWARD;
                            else {
                                latchedMove = NAV_WAIT;
                                sideClearanceStartTime = 0;
                                forwardBypassStartTime = 0;
                            }
                        }
                    }
                    else {
                        if (localCopy.us_left > 0 && localCopy.us_left < 40) computedCmd = NAV_STRAFE_RIGHT; 
                        else if (localCopy.us_right > 0 && localCopy.us_right < 40) computedCmd = NAV_STRAFE_LEFT; 
                    } 
                } 
            } 
        } 
        else { 
            computedCmd = localCopy.cmd;
        }

        if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
            sharedData.cmd = computedCmd;
            if (currentMissionStep != STEP_IDLE) sharedData.dist_to_target = distTraveledLeg;
            xSemaphoreGive(dataMutex);
        }

        if (!isInitializing) {
            if (xSemaphoreTake(i2cMutex, (TickType_t) 5) == pdTRUE) {
                display.update(localCopy.state, computedCmd, localCopy.us_front, localCopy.us_left, localCopy.us_right, localCopy.yaw, localCopy.dist_to_target);
                xSemaphoreGive(i2cMutex);
            }
        }
    } 
} 

// ==================================================================================
// SENSORS & DATA SENDING
// ==================================================================================
void sendBasicSensorData() {
    uint16_t f = normalizeUS(usFront.readDistance()); if (f == 0) f = 999;
    uint16_t l = normalizeUS(usLeft.readDistance());  if (l == 0) l = 999;
    uint16_t r = normalizeUS(usRight.readDistance()); if (r == 0) r = 999;
    uint16_t b = normalizeUS(usBack.readDistance());  if (b == 0) b = 999;
    uint16_t i = normalizeUS(usInclined.readDistance()); if (i == 0) i = 999;
    
    bool gpsValid = gps.locationValid();
    double lat = 0, lon = 0;
    
    if (gpsValid) {
        double rawLat = gps.getLatitude();
        double rawLon = gps.getLongitude();
        if (isFirstGpsReading) { filteredLat = rawLat; filteredLon = rawLon; isFirstGpsReading = false; }
        else {
            filteredLat = (GPS_FILTER_ALPHA * rawLat) + ((1.0 - GPS_FILTER_ALPHA) * filteredLat);
            filteredLon = (GPS_FILTER_ALPHA * rawLon) + ((1.0 - GPS_FILTER_ALPHA) * filteredLon);
        }
        lat = filteredLat; lon = filteredLon;
        
        float dec = wmm.magneticDeclination(lat, lon, WMM_YEAR, WMM_MONTH, WMM_DAY);
        if (dec > -30 && dec < 30) magneticDeclination = dec;
    }

    float roll = 0, pitch = 0, yaw = 0;
    static float prevYaw = 0.0;
    static float prevRoll = 0.0;
    static float prevPitch = 0.0;
    static int zeroCount = 0;

    if (!isInitializing) {
        if (xSemaphoreTake(i2cMutex, (TickType_t) 20) == pdTRUE) {
            imu2.updateCalibration();
            imu2.readOrientation(roll, pitch, yaw, magneticDeclination);
            xSemaphoreGive(i2cMutex);
        }
        
        if (abs(roll) < 0.01 && abs(pitch) < 0.01) {
             zeroCount++;
             if (zeroCount < 10) { 
                 yaw = prevYaw;
                 roll = prevRoll;
                 pitch = prevPitch;
             }
        } else {
             zeroCount = 0;
             prevYaw = yaw;
             prevRoll = roll;
             prevPitch = pitch;
        }
    }

    yaw -= YAW_NORTH_OFFSET;
    if (yaw < 0) yaw += 360; if (yaw >= 360) yaw -= 360;

    if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
        sharedData.us_front = f;
        sharedData.us_left = l;
        sharedData.us_right = r;
        sharedData.us_back = b;
        sharedData.us_incline = i;
        
        sharedData.yaw = yaw;
        sharedData.lat = lat;
        sharedData.lon = lon;
        sharedData.gps_valid = gpsValid;
        
        currentNavCommand = sharedData.cmd;
        xSemaphoreGive(dataMutex);
    }

    espNowData.us_front = f;
    espNowData.us_left = l;
    espNowData.us_right = r;
    espNowData.us_back = b;
    espNowData.us_incline = i;
    
    espNowData.imu_yaw = yaw;
    espNowData.imu_roll = roll;   
    espNowData.imu_pitch = pitch; 
    
    espNowData.gps_lat = lat;
    espNowData.gps_lon = lon;
    espNowData.gps_valid = gpsValid;
    espNowData.nav_command = (int8_t)currentNavCommand;
    
    espNowData.timestamp = millis();
    espNowData.gps_distance_to_monument = sharedData.dist_to_target; 
    
    logSensorData(); 
    esp_now_send(espCamAddress, (uint8_t *)&espNowData, sizeof(espNowData));
    lastBasicSendTime = millis();
}

void logSensorData()
{
    Serial.println("📡 Sensor Data:");
    Serial.printf("  US F:%d L:%d R:%d B:%d\n", espNowData.us_front, espNowData.us_left, espNowData.us_right, espNowData.us_back);
    Serial.printf("  Yaw: %.1f°\n", espNowData.imu_yaw);
    
    if (sharedData.is_mission_active) {
        Serial.printf("  [MISSION] Step: %d | Leg Dist: %.2fm | Target WP Index: %d\n", 
                      currentMissionStep, sharedData.dist_to_target, sharedData.current_wp_index);
    }

    if (espNowData.gps_lat != 0.0) {
        Serial.printf("  GPS: %.6f, %.6f\n", espNowData.gps_lat, espNowData.gps_lon);
    } else {
        Serial.println("  GPS: ❌ Invalid");
    }

    Serial.printf("  CMD: ");
    switch (currentNavCommand) {
        case NAV_WAIT: Serial.println("WAIT"); break;
        case NAV_TURN_LEFT: Serial.println("LEFT"); break;
        case NAV_TURN_RIGHT: Serial.println("RIGHT"); break;
        case NAV_FORWARD: Serial.println("FORWARD"); break;
        case NAV_STRAFE_LEFT: Serial.println("S_LEFT"); break;
        case NAV_STRAFE_RIGHT: Serial.println("S_RIGHT"); break;
        default: Serial.println("UNKNOWN"); break;
    }
    Serial.println("--------------------------");
}

void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(CommandMessage)) {
        memcpy(&receivedCmd, incomingData, sizeof(receivedCmd));
        Serial.printf("Received Cmd: %s\n", receivedCmd.command);
        if (strcmp(receivedCmd.command, "INIT") == 0) {
            triggerInitRoutine = true; 
            Serial.println(">>> INIT REQUEST RECEIVED. Queuing routine...");
        }
        else if (strcmp(receivedCmd.command, "POST") == 0) {
            bool isAtHBuilding = (sharedData.state == STATE_REACHED); 

            if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                if (isAtHBuilding) {
                     float cN, cE;
                     geo.toLocalCoordinates(sharedData.lat, sharedData.lon, cN, cE);
                     
                     int closestIdx = TOTAL_WAYPOINTS - 1; 
                     float minDistance = 100000.0f;
                     
                     for (int i = 0; i < TOTAL_WAYPOINTS; i++) {
                         float d = geo.getLocalDistance(cN, cE, localPath[i].n, localPath[i].e);
                         if (d < minDistance) {
                             minDistance = d;
                             closestIdx = i;
                         }
                     }
                     
                     sharedData.current_wp_index = closestIdx;
                     currentMissionStep = STEP_RETURN_NAV; 
                     currentMissionType = MISSION_RETURN;
                     sharedData.is_mission_active = true;
                     sharedData.state = STATE_WALKING;
                     
                     Serial.printf(">>> START RETURN MISSION. Found closest WP: %d (Dist: %.2fm)\n", closestIdx, minDistance);
                } else {
                     sharedData.is_mission_active = true;
                     sharedData.current_wp_index = 0;
                     currentMissionStep = STEP_BM1_ALIGN_EAST; 
                     sharedData.state = STATE_WALKING;
                     currentMissionType = MISSION_POST_ONLY;
                     Serial.println(">>> START POST MISSION: Fixed Point -> Blind 125 Align -> PAUSE");
                }
                xSemaphoreGive(dataMutex);
            }
        }
        else if (strcmp(receivedCmd.command, "H_BUILDING") == 0) {
            if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                if (currentMissionStep == STEP_BM1_PAUSED) {
                    currentMissionStep = STEP_BM1_FWD_LEG;
                    Serial.println(">>> RESUMING MISSION TO H_BUILDING");
                } 
                else if (sharedData.is_mission_active && currentMissionType == MISSION_POST_ONLY) {
                    currentMissionType = MISSION_FULL;
                    Serial.println(">>> UPGRADED MISSION TO FULL (H_BUILDING)");
                }
                else {
                    sharedData.current_wp_index = 0;
                    currentMissionStep = STEP_BM1_ALIGN_EAST; 
                    Serial.println(">>> START FULL H_BUILDING MISSION");
                }

                sharedData.is_mission_active = true;
                sharedData.state = STATE_WALKING;
                currentMissionType = MISSION_FULL;
                xSemaphoreGive(dataMutex);
            }
        }
        else if (strcmp(receivedCmd.command, "STOP") == 0) {
            isMissionPaused = !isMissionPaused; 

            if (isMissionPaused) {
                emergencyStopTriggered = true; 
                Serial.println(">>> PAUSED (Keeping Mission State)");
            }  
            else {
                emergencyStopTriggered = false; 
                Serial.println(">>> RESUMING Mission from last point...");
            }
        }
    }
}
void OnDataSent(const wifi_tx_info_t *mac, esp_now_send_status_t status) {}

void convertPathToLocal() {
    geo.setOrigin(WayPoints::MONUMENT_LAT, WayPoints::MONUMENT_LON);
    for (int i = 0; i < TOTAL_WAYPOINTS; i++) {
        geo.toLocalCoordinates(gpsPath[i].lat, gpsPath[i].lon, localPath[i].n, localPath[i].e);
    }
}