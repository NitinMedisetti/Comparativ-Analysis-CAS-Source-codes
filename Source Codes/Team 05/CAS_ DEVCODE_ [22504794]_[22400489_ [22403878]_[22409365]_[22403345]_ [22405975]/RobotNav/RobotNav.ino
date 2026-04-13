/*
 * FINAL CODE - Navigation System (Speed Optimized & Anti-Spin)
 * 1. Robot walks FORWARD for 8 seconds.
 * 2. Timer resets AFTER alignment (alignment time is not counted).
 * 3. Narrow Passage Logic (Segment 1): 2s wait on front, ignore sides if both triggered.
 * 4. Anti-Spin Logic: Shortest-path error calculation & actual-heading turn starts.
 * 5. VIRTUAL WALL REMOVED.
 */

#include <WiFi.h> 
#include <WebServer.h>
#include <TinyGPS++.h> 
#include <Wire.h> 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO055.h> 
#include <math.h>

#include "controller.h" 

// ==========================================
// 1. COORDINATE SYSTEM & CONSTANTS
// ==========================================
const double REF_LAT = 48.82987;
const double REF_LON = 12.95516;
const double SCALE = 600000.0;
const float COMPASS_OFFSET = 95.0; 
const float DECLINATION = 4.5;

const float DEADBAND = 15.0;           
const float ALIGNMENT_TOLERANCE = 7.0; 

struct Point2D { 
    float x; 
    float y; 
};

Point2D getXY(double lat, double lon) {
    Point2D p;
    p.x = (float)((lon - REF_LON) * SCALE);
    p.y = (float)((lat - REF_LAT) * SCALE);
    return p;
}

// ==========================================
// 2. PIN DEFINITIONS
// ==========================================
#define SDA_PIN 21
#define SCL_PIN 22
#define GPS_RX_PIN 16 
#define GPS_TX_PIN 17 

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 32 
#define OLED_RESET -1   
#define SCREEN_ADDRESS 0x3C 

#define TRIG_FRONT 32
#define ECHO_FRONT 25
#define TRIG_LEFT 4
#define ECHO_LEFT 18
#define TRIG_RIGHT 13
#define ECHO_RIGHT 19

const int OBSTACLE_MIN_CM = 0;                  
const int OBSTACLE_MAX_CM = 60;                 

const unsigned long ULTRASONIC_FRONT_STOP_DURATION_MS = 1000; 
const unsigned long ULTRASONIC_FRONT_STEP_DURATION_MS = 1000;    
const unsigned long ULTRASONIC_SIDE_AVOID_DURATION_MS = 1000;  
const unsigned long NARROW_PASSAGE_WAIT_MS = 2000; 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); 
TinyGPSPlus gps; 

// ==========================================
// 3. WAYPOINT DATA
// ==========================================
struct Waypoint {
    double latitude; 
    double longitude;
    const char* command; 
    float geofenceMeters; 
    unsigned long durationMs; 
};

const Waypoint waypointsToH[] = { 
  {48.829971, 12.955177, "TURN RIGHT", 6.0,  2500},
  {48.829929, 12.955075, "STEP LEFT", 4.0,  4500},
  {48.829540, 12.954758, "STOP",           4.0,  10000},
  {48.829503, 12.954718, "TURN RIGHT",  3.5,  2500},
  {48.829609, 12.954418, "TURN LEFT", 3.5,  2500},
  {48.829517, 12.954376, "STEP RIGHT", 4.0,  3000},
  {48.829490, 12.954289, "TURN RIGHT", 3.25,  2500}
};
const int NUM_WP_H = sizeof(waypointsToH) / sizeof(waypointsToH[0]);

// ==========================================
// 4. STATE VARIABLES
// ==========================================
int systemMode = 0; 
String unifiedMovementCommand = "WAITING..."; 
float currentHeading = 0; 
WebServer server(80);

bool avoidanceActive = false;
unsigned long avoidanceStartTime = 0;
const unsigned long AVOIDANCE_LOCK_DURATION = 800; 
String lastSafetyCommand = "NONE";

float referenceYawH = 0; 
bool rcPowerState = false;   
bool robotInitialized = false; 
const int DRIVE_SPEED = 28;  

int completedWaypointCountH = 0; 
String gpsNavigationCommandH = "WAITING"; 
bool isTurning90H = false; 
float turnStartingAngleH = 0; 

enum NavPhase { PHASE_WALKING, PHASE_ALIGNING };
NavPhase currentNavPhase = PHASE_ALIGNING; 
unsigned long navPhaseStartTime = 0;
const unsigned long WALK_SEGMENT_DURATION = 7000; 

String ultrasonicCommand = "NONE";             
unsigned long ultrasonicCommandStartTime = 0;   
String frontObstacleSequenceState = "NONE"; 

long frontDistCm = 999; 
long leftDistCm = 999;  
long rightDistCm = 999; 

// ==========================================
// 5. SENSOR FUNCTIONS
// ==========================================
long readUltrasonicDistance(int trigPin, int echoPin) {
    long readings[3]; 
    for (int i = 0; i < 3; i++) {
        digitalWrite(trigPin, LOW); 
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH); 
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        long duration = pulseIn(echoPin, HIGH, 30000); 
        long distance = 999;
        if (duration > 0) distance = duration * 0.034 / 2;
        readings[i] = distance;
        delay(5); 
    }
    // Simple median filter for 3 values
    if (readings[0] > readings[1]) { long temp = readings[0]; readings[0] = readings[1]; readings[1] = temp; }
    if (readings[1] > readings[2]) { long temp = readings[1]; readings[1] = readings[2]; readings[2] = temp; }
    if (readings[0] > readings[1]) { long temp = readings[0]; readings[0] = readings[1]; readings[1] = temp; }
    return readings[1];
}

void readAllSensors() {
    frontDistCm = readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
    leftDistCm = readUltrasonicDistance(TRIG_LEFT, ECHO_LEFT);
    rightDistCm = readUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT);
}

// ==========================================
// 6. NAVIGATION & OBSTACLE MODULES
// ==========================================
void checkUltrasonicObstacles() {
    bool leftWarning  = (leftDistCm >= OBSTACLE_MIN_CM && leftDistCm <= OBSTACLE_MAX_CM);
    bool rightWarning = (rightDistCm >= OBSTACLE_MIN_CM && rightDistCm <= OBSTACLE_MAX_CM);
    bool frontWarning = (frontDistCm >= OBSTACLE_MIN_CM && frontDistCm <= OBSTACLE_MAX_CM);

    // Narrow Passage Logic for Segment 1
    if (completedWaypointCountH == 0) {
        if (leftWarning && rightWarning) {
            if (frontWarning) {
                if (frontObstacleSequenceState != "NARROW_STOP") {
                    ultrasonicCommand = "STOP";
                    frontObstacleSequenceState = "NARROW_STOP";
                    ultrasonicCommandStartTime = millis();
                } else {
                    if (millis() - ultrasonicCommandStartTime >= NARROW_PASSAGE_WAIT_MS) {
                        if (frontWarning) {
                            ultrasonicCommand = "STOP";
                        } else { 
                            ultrasonicCommand = "NONE"; 
                            frontObstacleSequenceState = "NONE"; 
                        }
                    }
                }
                return;
            } else { 
                ultrasonicCommand = "NONE"; 
                frontObstacleSequenceState = "NONE"; 
                return; 
            }
        }
    }

    // Front Obstacle Sequence (Stop then Step)
    if (frontObstacleSequenceState == "STOP_PHASE") {
        if (millis() - ultrasonicCommandStartTime >= ULTRASONIC_FRONT_STOP_DURATION_MS) {
            if (leftDistCm <= OBSTACLE_MAX_CM) {
                ultrasonicCommand = "STEP RIGHT";
            } else {
                ultrasonicCommand = "STEP LEFT";
            }
            frontObstacleSequenceState = "STEP_EXEC_PHASE"; 
            ultrasonicCommandStartTime = millis();
        }
        return; 
    }

    if (frontObstacleSequenceState == "STEP_EXEC_PHASE") {
        if (millis() - ultrasonicCommandStartTime >= ULTRASONIC_FRONT_STEP_DURATION_MS) {
            ultrasonicCommand = "NONE"; 
            frontObstacleSequenceState = "NONE";
        }
        return; 
    }

    // Side avoidance logic
    if (ultrasonicCommand == "STEP RIGHT" || ultrasonicCommand == "STEP LEFT") {
        if (millis() - ultrasonicCommandStartTime >= ULTRASONIC_SIDE_AVOID_DURATION_MS) {
            ultrasonicCommand = "NONE"; 
        } else {
            return;
        }
    }

    if (frontWarning) {
        ultrasonicCommand = "STOP"; 
        frontObstacleSequenceState = "STOP_PHASE";
        ultrasonicCommandStartTime = millis();
    } 
    else if (leftWarning && rightWarning) { 
        ultrasonicCommand = "NONE"; 
    }
    else if (leftWarning) { 
        ultrasonicCommand = "STEP RIGHT"; 
        ultrasonicCommandStartTime = millis(); 
    } 
    else if (rightWarning) { 
        ultrasonicCommand = "STEP LEFT"; 
        ultrasonicCommandStartTime = millis(); 
    }
    else { 
        ultrasonicCommand = "NONE"; 
    }
}

void checkNavigationH() {
    if (gps.hdop.isValid() && gps.hdop.hdop() > 2.5) return;

    // --- Turning Logic (90 Degree Turns) ---
    if (isTurning90H) {
        Waypoint currentWP = waypointsToH[completedWaypointCountH];
        String cmdStr = String(currentWP.command);

        if (cmdStr.indexOf("TURN") != -1) {
            float diff = abs(currentHeading - turnStartingAngleH); 
            if (diff > 180) diff = 360 - diff;

            if (diff >= 84.0) { 
                isTurning90H = false;
                if (cmdStr.indexOf("RIGHT") != -1) referenceYawH += 90;
                else if (cmdStr.indexOf("LEFT") != -1) referenceYawH -= 90;
                
                if (referenceYawH >= 360) referenceYawH -= 360;
                if (referenceYawH < 0) referenceYawH += 360;
                
                completedWaypointCountH++; 
                currentNavPhase = PHASE_ALIGNING;
            }
        } else {
            // Logic for "STOP" or duration based commands
            if (millis() - (unsigned long)turnStartingAngleH >= currentWP.durationMs) { 
                isTurning90H = false; 
                completedWaypointCountH++; 
                currentNavPhase = PHASE_ALIGNING;
            }
        }
        return; 
    }

    // --- Main Geofencing & Phased Navigation ---
    if (completedWaypointCountH < NUM_WP_H) {
        Waypoint target = waypointsToH[completedWaypointCountH];
        Point2D curPos = getXY(gps.location.lat(), gps.location.lng());
        Point2D tarPos = getXY(target.latitude, target.longitude);
        float distUnits = sqrt(pow(tarPos.x - curPos.x, 2) + pow(tarPos.y - curPos.y, 2));
        
        // Check if inside geofence
        if (distUnits <= (target.geofenceMeters * 5.4)) {
            isTurning90H = true; 
            gpsNavigationCommandH = target.command;
            
            if (String(target.command).indexOf("TURN") != -1) {
                turnStartingAngleH = currentHeading; 
            } else {
                turnStartingAngleH = (float)millis(); 
            }
            return;
        }

        // --- ANTI-SPIN: SHORTEST PATH CALCULATION ---
        float error = referenceYawH - currentHeading;
        if (error < -180) error += 360;
        if (error > 180) error -= 360;

        if (currentNavPhase == PHASE_WALKING) {
            if (millis() - navPhaseStartTime < WALK_SEGMENT_DURATION) {
                gpsNavigationCommandH = "FORWARD";
                // Drift correction
                if (abs(error) > DEADBAND) {
                     gpsNavigationCommandH = "STOP";
                     currentNavPhase = PHASE_ALIGNING;
                }
            } else {
                gpsNavigationCommandH = "STOP";
                currentNavPhase = PHASE_ALIGNING;
            }
        } 
        else if (currentNavPhase == PHASE_ALIGNING) {
            if (abs(error) <= ALIGNMENT_TOLERANCE) {
                currentNavPhase = PHASE_WALKING;
                navPhaseStartTime = millis(); 
                gpsNavigationCommandH = "FORWARD"; 
            } else {
                if (error > 0) gpsNavigationCommandH = "TURN RIGHT";
                else gpsNavigationCommandH = "TURN LEFT";
            }
        }
    } else { 
        gpsNavigationCommandH = "STOP"; 
    }
}

// ==========================================
// 7. WEB SERVER & HARDWARE UI
// ==========================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: sans-serif; text-align: center; background: #111; color: white; }
  .box { border: 2px solid #00E676; padding: 20px; margin: 20px; border-radius: 15px; }
  button { padding: 15px; font-size: 18px; width: 80%; margin: 10px auto; border-radius: 5px; border: none; font-weight: bold; cursor: pointer; display: block; }
  .btn-h { background-color: #2196F3; color: white; }
  .btn-stop { background-color: #f44336; color: white; }
  .btn-rc { background-color: #9C27B0; color: white; }
  .btn-init { background-color: #00E676; color: black; }
  .disabled { background-color: #555 !important; cursor: not-allowed; color: #888; pointer-events: none; }
</style></head>
<body>
  <h1>ROBOT DOG SPEED-NAV PRO</h1>
  <div class="box">
    <div id="mode-status">STOPPED</div>
    <div id="cmd" style="font-size: 26px; color: #00E676;">WAITING...</div>
    <div style="font-size: 40px;"><span id="ang">0</span>&deg;</div>
  </div>
  <div style="padding: 10px;">
    <button id="btn-rc" class="btn-rc disabled" onclick="toggleRC()">RC POWER</button>
    <button id="btn-init" class="btn-init" onclick="initializeRobot()">INITIALIZE</button>
  </div>
  <button id="btn-h" class="btn-h disabled" onclick="setMode(1)">GO TO H</button>
  <button id="btn-stop" class="btn-stop disabled" onclick="setMode(0)">STOP SYSTEM</button>
  <script>
    function setMode(mode) { fetch('/setMode?val=' + mode); }
    function toggleRC() { fetch('/toggleRC'); }
    function initializeRobot() { document.getElementById('btn-init').innerText = "INITIALIZING..."; fetch('/initialize'); }
    setInterval(() => {
      fetch('/status').then(r => r.json()).then(d => {
        document.getElementById('cmd').innerText = d.command;
        document.getElementById('ang').innerText = Math.round(d.head);
        document.getElementById("mode-status").innerText = (d.mode == 1) ? "NAVIGATING TO H" : "STOPPED";
        const btnRc = document.getElementById('btn-rc');
        const btnInit = document.getElementById('btn-init');
        const btnH = document.getElementById('btn-h');
        const btnStop = document.getElementById('btn-stop');
        if (!d.init) {
            btnRc.classList.add('disabled'); btnH.classList.add('disabled'); btnStop.classList.add('disabled');
            btnInit.classList.remove('disabled'); btnInit.innerText = "INITIALIZE";
        } else {
            btnRc.classList.remove('disabled'); btnRc.innerText = d.rc ? "RC POWER (ON)" : "RC POWER (OFF)";
            if(d.mode == 0) { btnInit.innerText = "RE-INITIALIZE"; btnInit.classList.remove('disabled'); }
            else { btnInit.innerText = "RUNNING..."; btnInit.classList.add('disabled'); }
            if(d.rc) { btnH.classList.remove('disabled'); btnStop.classList.remove('disabled'); }
            else { btnH.classList.add('disabled'); btnStop.classList.add('disabled'); }
        }
      });
    }, 500);
  </script>
</body></html>)rawliteral";

void handleSetMode() {
  if (server.hasArg("val")) {
    systemMode = server.arg("val").toInt();
    if (systemMode == 1) { 
        referenceYawH = currentHeading; 
        currentNavPhase = PHASE_ALIGNING; 
    }
    completedWaypointCountH = 0; 
    isTurning90H = false; 
    ultrasonicCommand = "NONE"; 
    frontObstacleSequenceState = "NONE";
  }
  server.send(200, "text/plain", "OK");
}

void handleToggleRC() { 
    rcPowerState = !rcPowerState; 
    if(!rcPowerState) stop(); 
    RConoffswitch(); 
    server.send(200, "text/plain", "OK"); 
}

void handleInitialize() { 
    server.send(200, "text/plain", "Initializing..."); 
    initialcheckuproutine(); 
    robotInitialized = true; 
    rcPowerState = true; 
}

void handleStatus() {
  String json = "{\"mode\":" + String(systemMode) + ",\"command\":\"" + unifiedMovementCommand + 
                "\",\"head\":" + String(currentHeading) + ",\"rc\":" + (rcPowerState ? "true" : "false") +
                ",\"init\":" + (robotInitialized ? "true" : "false") + ",\"f\":" + String(frontDistCm) + "}";
  server.send(200, "application/json", json);
}

void applyPhysicalMovement() {
    if (!rcPowerState || !robotInitialized) { stop(); return; }
    if (unifiedMovementCommand == "FORWARD") forward(DRIVE_SPEED);
    else if (unifiedMovementCommand == "TURN LEFT") rotateLeft(DRIVE_SPEED);
    else if (unifiedMovementCommand == "TURN RIGHT") rotateRight(DRIVE_SPEED);
    else if (unifiedMovementCommand == "STEP LEFT") stepLeft(DRIVE_SPEED);
    else if (unifiedMovementCommand == "STEP RIGHT") stepRight(DRIVE_SPEED);
    else stop();
}

// ==========================================
// 8. SETUP & LOOP
// ==========================================
void setup() {
    Serial.begin(115200); 
    Wire.begin(SDA_PIN, SCL_PIN);
    
    initExpMod(); 
    initDAC();
    
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { for(;;); }
    display.clearDisplay(); 
    display.display();
    
    if (!bno.begin()) { while(1); }
    bno.setExtCrystalUse(true);
    
    pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
    
    Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    WiFi.softAP("ESP32_NAV_DOG", "12345678");
    server.on("/", [](){ server.send(200, "text/html", index_html); });
    server.on("/setMode", handleSetMode);
    server.on("/status", handleStatus);
    server.on("/toggleRC", handleToggleRC);
    server.on("/initialize", handleInitialize);
    server.begin();
}

void loop() {
    server.handleClient();
    while (Serial1.available() > 0) gps.encode(Serial1.read());

    // Compass calculation
    sensors_event_t event; 
    bno.getEvent(&event);
    float adjusted = event.orientation.x - COMPASS_OFFSET;
    if (adjusted < 0) adjusted += 360;
    float calc = adjusted + DECLINATION;
    if (calc >= 360) calc -= 360;
    if (calc < 0) calc += 360;
    currentHeading = calc;

    readAllSensors(); 
    if (systemMode == 1) { 
        checkNavigationH(); 
    }
    checkUltrasonicObstacles();

    // Command Prioritization Logic
    if (ultrasonicCommand != "NONE") {
        unifiedMovementCommand = ultrasonicCommand; 
        lastSafetyCommand = ultrasonicCommand;
        avoidanceActive = true; 
        avoidanceStartTime = millis();
    } 
    else if (avoidanceActive && (millis() - avoidanceStartTime < AVOIDANCE_LOCK_DURATION)) {
        unifiedMovementCommand = lastSafetyCommand; 
    }
    else {
        avoidanceActive = false; 
        if (systemMode == 1) {
            unifiedMovementCommand = gpsNavigationCommandH;
        } else {
            unifiedMovementCommand = "STOPPED";
        }
    }

    applyPhysicalMovement();

    // OLED Update
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    if (systemMode != 0) {
        display.setTextSize(1);
        display.print(currentNavPhase == PHASE_WALKING ? "WALK" : "ALIGN");
        display.setCursor(50, 0);
        display.print("H:" + String((int)currentHeading));
        display.setTextSize(2); 
        display.setCursor(0, 18); 
        display.print(unifiedMovementCommand); 
    } else {
        display.setTextSize(1);
        display.print(robotInitialized ? (rcPowerState ? "READY" : "RC OFF") : "INITIALIZE");
    }
    display.display();
    
    delay(10); 
}