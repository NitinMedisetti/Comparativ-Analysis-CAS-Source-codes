/*
CAS_source_22409080_22400891_22402106_22409003_22502391_22505577_22519633
Team 6 
Case Study Cooperative and Autonomous Systems Source Code

Team Members:
Bharadwaj Koorapati - 22409080
Akhil Dodla - 22400891
Sreedev Mathoor Valappil Jayaraj - 22519633
Nikhil Raacharla - 22402106
Ashish Uriviti - 22409003
Vinay Yadav - 22502391
Krunal Bele - 22505577

*/
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO055.h>
#include "Controller.h"

//Pins
#define FRONT_TRIG 32
#define FRONT_ECHO 25
#define RIGHT_TRIG 13
#define RIGHT_ECHO 19
#define LEFT_TRIG 4
#define LEFT_ECHO 18
#define BACK_TRIG 5
#define BACK_ECHO 14
#define FRONT45_TRIG 23
#define FRONT45_ECHO 26

//Configuration

const char* ap_ssid = "Team6_DogWifi";      // Change this name
const char* ap_password = "team6rocks";  // Change this (8+ chars)
const float TOTAL_NORTH_OFFSET = -90.2;
const float OBS_THRESHOLD = 150.0;
const float SIDE_OBS_THRESHOLD = 80.0;
const float RADIUS_INTERMEDIATE = 4.0;
const float RADIUS_FINAL = 3.5;
const char* csvPath = "/log.csv";

//Motor Control Configuration
const int FORWARD_SPEED = 28;
const int TURN_SPEED = 28;
const int BACKWARD_SPEED = 28;
const int STEP_SPEED = 28;

//Obstacle Avoidance Timer
unsigned long clearanceTimer = 0;
const int FORCED_FORWARD_MS = 6000;

//Logging Configuration
unsigned long lastLogTime = 0;
const int LOG_INTERVAL_MS = 500; // Log data 2 times per second (Adjustable)
bool missionended = false;

// Global State
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);
WebServer server(80);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// POSTURE TRACKING
// 0 = Standing (Default state after checkup)
// 1 = Locked
// 2 = Laying Down
int postureState = 0; 

// HOME CONFIGURATION
const float HOME_HEADING = 19.0; // Heading to face when docking (0=North)

//State Machine
enum MissionPhase { 
  HIT_D_BUILD, 
  TURN_TO_D_LEFT, 
  FIND_CHANNEL, 
  TURN_TO_CHANNEL, 
  K_TURN_RIGHT, 
  END_MISSION, 
  ORIENT_NORTH, 
  HOME_TURN,      
  HIT_K_BUILD, 
  WALL_FOLLOW_CHANNEL
};

double curLat, curLon, tarLat, tarLon, bearing, distance;
float distF, distL, distR, trueYaw;
uint8_t m_cal = 0;
bool navStarted = false;
bool loggingActive = false;
bool imuInitialized = false;
bool imuCalibrated = false;
bool calibrationOverride = false;
bool hardwareOverride = false;
bool checkupCompleted = false;
bool expModReady = false;
bool dacReady = false;
bool hardwareChecked = false;
bool returnHomeMode = false;
int wpIdx = 0;

enum MoveCmd { CMD_IDLE, CMD_LEFT, CMD_RIGHT, CMD_FWD, CMD_BACK, CMD_STEP_LEFT, CMD_STEP_RIGHT };
MoveCmd currentCmd = CMD_IDLE;
const char* cmdNames[] = {"STOPPED", "TURN LEFT", "TURN RIGHT", "FORWARD", "BACKWARD", "STEP LEFT", "STEP RIGHT"};

MissionPhase currentPhase = HIT_D_BUILD;
unsigned long finalMoveTimer = 0;

struct WP { double lat; double lon; };
const WP path[] = {
  {48.829960, 12.955153}, {48.829933, 12.955117}, {48.829855, 12.95507},
  {48.829784, 12.954989}, {48.829623, 12.954838}, {48.8295445, 12.954786},
  {48.829463, 12.954659}, {48.829350, 12.954536},
  {48.829362, 12.954451}, {48.829428, 12.954302}, {48.829470, 12.954361}
};
const int NUM_WP = sizeof(path) / sizeof(path[0]);

//HOME POINT
const WP homePoint = {48.830023, 12.954921};

float getDistFast(int tPin, int ePin) {
  digitalWrite(tPin, LOW); delayMicroseconds(2);
  digitalWrite(tPin, HIGH); delayMicroseconds(10);
  digitalWrite(tPin, LOW);
  long duration = pulseIn(ePin, HIGH, 15000UL);
  return (duration == 0) ? 400.0 : duration * 0.0343 / 2.0;
}

void configureGPS() {
  byte setBaud[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x93, 0x90};
  GPS_Serial.write(setBaud, sizeof(setBaud));
  delay(100);
  GPS_Serial.begin(38400, SERIAL_8N1, 16, 17);
  byte set5Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  GPS_Serial.write(set5Hz, sizeof(set5Hz));
}

void executeMotorCommand() {
  static MoveCmd lastCmd = CMD_IDLE;
  static unsigned long cmdTimer = 0;
  const unsigned long CMD_HOLD_MS = 500;

  if (currentCmd != lastCmd && (cmdTimer == 0 || millis() - cmdTimer > CMD_HOLD_MS)) {
    lastCmd = currentCmd;
    cmdTimer = millis();

    switch(currentCmd) {
      case CMD_IDLE:
        stop();
        Serial.println("Motor: STOP");
        break;
      case CMD_FWD:
        forward(FORWARD_SPEED);
        Serial.println("Motor: FORWARD");
        break;
      case CMD_BACK:
        backward(BACKWARD_SPEED);
        Serial.println("Motor: BACKWARD");
        break;
      case CMD_LEFT:
        rotateLeft(TURN_SPEED);
        Serial.println("Motor: ROTATE LEFT");
        break;
      case CMD_RIGHT:
        rotateRight(TURN_SPEED);
        Serial.println("Motor: ROTATE RIGHT");
        break;
      case CMD_STEP_LEFT:
        stepLeft(STEP_SPEED);
        Serial.println("Motor: STEP LEFT");
        break;
      case CMD_STEP_RIGHT:
        stepRight(STEP_SPEED);
        Serial.println("Motor: STEP RIGHT");
        break;
    }
  }
}

//ZONE DEFINITIONS
const WP ZONE_1 = {48.829904, 12.954846};
const WP ZONE_2 = {48.82944, 12.95447};
const WP FINAL_PT = {48.829470, 12.954360};


//  GLOBAL VARIABLES FOR NAVIGATION LOGIC

unsigned long phase1BlockTimer = 0;   
unsigned long phase2GapTimer = 0;     
unsigned long phase3ExitTimer = 0;    
unsigned long phase4BlockTimer = 0;   
unsigned long phase5BlockTimer = 0;   
unsigned long phase5LostWallTimer = 0;

float phase2TargetHeading = 0.0;      

// Phase 6 Variables
int p6SubState = 0;           // 0=Search, 1=Verify, 2=Post-Verify, 3=Turn, 4=FinalApproach
int p6VerifyCount = 0;        // Counts the Fwd/Back motions
unsigned long p6Timer = 0;    // General timer for Phase 6 moves
float p6TurnTarget = 0.0;     // Stores the 90-degree turn target


//  FULL MISSION NAVIGATION LOGIC

void updateNavigation() {
  if (!navStarted) return;

  // 1. READ SENSORS
  distF = getDistFast(FRONT_TRIG, FRONT_ECHO);
  distL = getDistFast(LEFT_TRIG, LEFT_ECHO);
  distR = getDistFast(RIGHT_TRIG, RIGHT_ECHO);
  double distToFinal = TinyGPSPlus::distanceBetween(curLat, curLon, FINAL_PT.lat, FINAL_PT.lon);

  switch (currentPhase) {
    // PHASE 1: APPROACH
    case ORIENT_NORTH:
    {
      Serial.println("Oriented to North");
    }
  //Zone1_Trigger
    case HIT_D_BUILD:
      {
          bool isBlocked = (distF < 60.0); 

          if (isBlocked) {
              if (phase1BlockTimer == 0) phase1BlockTimer = millis();

              if (millis() - phase1BlockTimer > 10000) {
                  phase2TargetHeading = angleDiff(213.0, 70.0); 
                  if (phase2TargetHeading < 0) phase2TargetHeading += 360;
                  
                  currentPhase = TURN_TO_D_LEFT; 
                  phase1BlockTimer = 0;
                  Serial.println("P1: Wall Confirmed. Turning Left 90°.");
                  return;
              }
              if(distR < 100.0) 
              {
                currentCmd = CMD_STEP_LEFT; 
              }
              else{
              currentCmd = CMD_STEP_RIGHT; 
              }
            } 
          else {
              phase1BlockTimer = 0; 
              float target = 218.0; 
              float diff = angleDiff(target, trueYaw);
              if (abs(diff) > 6.0) currentCmd = (diff > 0) ? CMD_RIGHT : CMD_LEFT;
              else currentCmd = CMD_FWD;
          }

          if(distR < 20.0)
          {
            currentCmd = CMD_STEP_LEFT; 
          }
          if (distL < 20.0)
          {
            currentCmd = CMD_STEP_RIGHT;
          }
          
      }
      break;

    // TRANSITION: ROTATE LEFT 90
    case TURN_TO_D_LEFT: 
      {
          float diff = angleDiff(phase2TargetHeading, trueYaw);
          if (abs(diff) < 6.0) {
              currentPhase = FIND_CHANNEL; 
              phase2GapTimer = 0;
              Serial.println("P2 START: Wall Follow & Gap Hunt");
          } else {
              currentCmd = CMD_LEFT; 
          }
      }
      break;

  
    // Zone_2: WALL FOLLOW & GAP IDENTIFICATION
  
    case FIND_CHANNEL: 
      {
          if (distF < 60.0) {
              phase2GapTimer = 0; 
              if(distR < 120.0){
              currentCmd = CMD_STEP_LEFT; 
              }
              else{
              currentCmd = CMD_STEP_RIGHT; 
              }
              return;
          }

          if (distR > 200.0) {
              if (phase2GapTimer == 0) phase2GapTimer = millis();
              if (millis() - phase2GapTimer > 4000) {
                  currentPhase = TURN_TO_CHANNEL; 
                  phase2GapTimer = 0;
                  Serial.println("P2: Gap Confirmed. Turning 218° to Enter.");
                  return;
              }
              currentCmd = CMD_FWD;
              return;
          } else {
              phase2GapTimer = 0;
          }

          if (distR < 60.0) currentCmd = CMD_STEP_LEFT; 
          else if (distR > 125.0) currentCmd = CMD_STEP_RIGHT; 
          else currentCmd = CMD_FWD; 
      }
      break;

    //TRANSITION: TURN TO 218
    case TURN_TO_CHANNEL: 
      {
          float target = 213.0; 
          float diff = angleDiff(target, trueYaw);
          if (abs(diff) < 5.0) {
              currentPhase = WALL_FOLLOW_CHANNEL; 
              phase3ExitTimer = 0;
              Serial.println(">>> P3 START: Channel Maneuver <<<");
          } else {
              currentCmd = CMD_RIGHT; 
          }
      }
      break;

  
    //Zone_3 : CHANNEL MANEUVER
  
    case WALL_FOLLOW_CHANNEL: 
      {
          if (distR > 200.0) {
              if (phase3ExitTimer == 0) phase3ExitTimer = millis();
              if (millis() - phase3ExitTimer > 8000) {
                  currentPhase = HIT_K_BUILD; 
                  Serial.println("P3 COMPLETE: Exited Channel.");
                  return;
              }
          } else {
              phase3ExitTimer = 0;
          }

          if (distF < 60.0) {
              currentCmd = CMD_STEP_LEFT;
              return;
          }

          float target = 213.0;
          float diff = angleDiff(target, trueYaw);
          
          if (distR < 200.0) { 
              if (distR < 70.0) currentCmd = CMD_STEP_LEFT; 
              else if (distR > 100.0) currentCmd = CMD_STEP_RIGHT; 
              else {
                  if (abs(diff) > 6.0) currentCmd = (diff > 0) ? CMD_RIGHT : CMD_LEFT;
                  else currentCmd = CMD_FWD;
              }
          } else {
              if (abs(diff) > 6.0) currentCmd = (diff > 0) ? CMD_RIGHT : CMD_LEFT;
              else currentCmd = CMD_FWD;
          }
      }
      break;
    case HIT_K_BUILD: 
      {
          bool isBlocked = (distF < 60.0);

          if (isBlocked) {
              if (phase4BlockTimer == 0) phase4BlockTimer = millis();

              if (millis() - phase4BlockTimer > 6000) {
                  currentPhase = K_TURN_RIGHT; 
                  phase4BlockTimer = 0;
                  Serial.println("P4: Wall Confirmed. Turning 290°.");
                  return;
              }
              if(distR < 100.0) 
              {
                currentCmd = CMD_STEP_LEFT; 
              }
              else{
              currentCmd = CMD_STEP_RIGHT; 
              }
              Serial.printf("P4: Dodging... (%lums)\n", millis() - phase4BlockTimer);
          } 
          else {
              phase4BlockTimer = 0;
              // OLD HEADINGS:225 - Went too left to the stairs
              float target = 235.0; 
              float diff = angleDiff(target, trueYaw);
              if (abs(diff) > 5.0) currentCmd = (diff > 0) ? CMD_RIGHT : CMD_LEFT;
              else currentCmd = CMD_FWD;
          }
      }
      break;

  
    // Zone_4: HYBRID WALL FOLLOW & TURN 60°
  
    case K_TURN_RIGHT: 
      {
         // A. TURN TO 290
         static bool turnedTo290 = false;
         if (!turnedTo290) {
             float target = 300.0;
             float diff = angleDiff(target, trueYaw);
             if (abs(diff) < 5.0) {
                 turnedTo290 = true;
                 phase5BlockTimer = 0;
                 phase5LostWallTimer = 0;
                 Serial.println("P5 START: Hybrid Wall Follow");
             } else {
                 currentCmd = CMD_RIGHT;
                 return;
             }
         }

         // B. STOP TRIGGER (Front < 1.8m for > 3s)
         if (distF < 180.0) {
             if (phase5BlockTimer == 0) phase5BlockTimer = millis();
             if (millis() - phase5BlockTimer > 3000) {
                 // SWITCH TO PHASE 6 (END_MISSION State)
                 currentPhase = END_MISSION; 
                 p6SubState = 0; // Initialize Phase 6
                 Serial.println("P5 COMPLETE: Turning 60°.");
                 turnedTo290 = false; 
                 return;
             }
             // CONFIRM WITH STEP RIGHT
             currentCmd = CMD_STEP_RIGHT; 
             Serial.println("P5: Verifying Wall (Step Right)... ");
             return;
         } else {
             phase5BlockTimer = 0;
         }

         // C. HYBRID WALL FOLLOWING (Left Wall @ 1.5m)
         if (distL < 200.0) {
             phase5LostWallTimer = 0;
             if (distL < 130.0) currentCmd = CMD_STEP_RIGHT; 
             else if (distL > 170.0) currentCmd = CMD_STEP_LEFT; 
             else currentCmd = CMD_FWD; 
         } 
         else {
             if (phase5LostWallTimer == 0) phase5LostWallTimer = millis();
             if (millis() - phase5LostWallTimer > 5000) {
                 float target = 300.0;
                 float diff = angleDiff(target, trueYaw);
                 if (abs(diff) > 5.0) currentCmd = (diff > 0) ? CMD_RIGHT : CMD_LEFT;
                 else currentCmd = CMD_FWD;
             } else {
                 currentCmd = CMD_FWD;
             }
         }
      }
      break;

  
    // Zone_5: BULLSEYE
  
    case END_MISSION:
      {
    static unsigned long mcStartTime = 0;


    static bool turnedTo15 = false;
         if (!turnedTo15) {
             float target = 15.0;
             float diff = angleDiff(target, trueYaw);
             if (abs(diff) < 5.0) {
                  turnedTo15 = true;
                  mcStartTime = 0; // Reset timer for wall-follow
                  Serial.println(">>> P6 START: Hybrid Wall Follow <<<");
             } else {
                 currentCmd = CMD_RIGHT;
                 return;
             }
         }
    // Start timer once
    if (mcStartTime == 0) {
        mcStartTime = millis();
        Serial.println("P6 START Simple wall-follow forward for 5s");
    }

    // 15 seconds total
    if (millis() - mcStartTime >= 15000) {
        currentCmd = CMD_IDLE;
        navStarted = false;
        missionended = true;
        Serial.println("P6 DONE. Mission complete.");
        mcStartTime = 0;
        break;
    }

    // Simple left-wall-follow while moving forward
    if (distL < 130.0) {
        currentCmd = CMD_STEP_RIGHT;   // too close to left wall --> move right
    } else if (distL > 170.0) {
        currentCmd = CMD_STEP_LEFT;    // too far from left wall --> move left
    } else {
        currentCmd = CMD_FWD;         // in band --> go forward
    }
      break;
  }
  }
}


const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><style>
  body { font-family: sans-serif; background: #111; color: white; text-align: center; padding: 10px; }
  .card { background: #222; padding: 15px; border-radius: 10px; margin: 10px auto; border: 1px solid #444; max-width: 500px; }
  .compass { position: relative; width: 180px; height: 180px; border: 3px solid #555; border-radius: 50%; margin: 15px auto; }
  .needle { position: absolute; left: 50%; transform-origin: bottom center; transition: 0.2s; }
  .target { top: 10px; height: 80px; width: 4px; background: #ff3e3e; }
  .robot { top: 30px; height: 60px; width: 6px; background: #3e92ff; }
  button { padding:12px; margin:8px; width:90%; max-width:450px; border-radius:5px; border:none; font-weight:bold; font-size:1em; cursor:pointer; }
  button:disabled { opacity: 0.5; cursor: not-allowed; }
  .btn-imu { background:#9c27b0; color:white; }
  .btn-hw-check { background:#17a2b8; color:white; }
  .btn-checkup { background:#ffc107; color:#000; }
  .btn-start { background:#28a745; color:white; }
  .btn-home { background:#ff9800; color:white; }
  .btn-stop { background:#dc3545; color:white; font-size:1.1em; }
  .btn-override { background:#ff5722; color:white; border: 2px solid #ffeb3b; }
  .status-box { padding: 12px; margin: 10px auto; border-radius: 5px; font-weight: bold; max-width: 450px; }
  .status-pending { background: #ff9800; color: black; }
  .status-ready { background: #4caf50; color: white; }
  .status-error { background: #f44336; color: white; }
  .status-progress { background: #2196f3; color: white; }
  .status-override { background: #ff5722; color: white; border: 2px solid #ffeb3b; }
  .hw-status { display: inline-block; margin: 5px; padding: 8px; border-radius: 5px; font-size: 0.9em; }
  .init-sequence { background: #333; padding: 15px; margin: 15px auto; border-radius: 10px; max-width: 450px; border: 2px solid #555; }
  .step-title { font-size: 1.1em; font-weight: bold; color: #ffc107; margin-bottom: 10px; }
  .override-section { background: #1a1a1a; padding: 10px; margin: 10px auto; border-radius: 8px; max-width: 450px; border: 2px dashed #ff5722; }
  .warning-text { color: #ffeb3b; font-size: 0.9em; margin: 5px; }
</style></head><body>
  <h2> Rover Command Center</h2>
  
  
  <div class="init-sequence">
    <div class="step-title"> Initialization Sequence</div>
    
    <!-- Step 1: IMU -->
    <div id="imuSection" class="status-box status-pending">
      1 IMU Initialization Required
    </div>
    <button class="btn-imu" id="imuBtn" onclick="initIMU()"> STEP 1: INITIALIZE & CALIBRATE IMU</button>
    <div id="imuDetails" style="display:none; margin: 10px; color: #aaa;">
      Magnetometer Calibration: <span id="magCal" style="color:#ffc107; font-weight:bold;">0/3</span>
    </div>
    
    <!-- Step 2: Hardware Check -->
    <div id="hwSection" class="status-box status-pending" style="display:none;">
      2 Hardware Check Required
    </div>
    <button class="btn-hw-check" id="hwCheckBtn" onclick="checkHardware()" style="display:none;"> STEP 2: CHECK DAC & I/O EXPANDER</button>
    <div id="hwDetails" style="display:none; margin: 10px;">
      <span id="expModStatus" class="hw-status">MCP23017: ?</span>
      <span id="dacStatus" class="hw-status">MCP4728: ?</span>
    </div>
    
    <!-- Step 3: Motor Checkup -->
    <div id="checkupSection" class="status-box status-pending" style="display:none;">
      3 Motor Checkup Required
    </div>
    <button class="btn-checkup" id="checkupBtn" onclick="runCheckup()" style="display:none;"> STEP 3: RUN MOTOR CHECKUP</button>
  </div>
  
  <div id="cmd" style="font-size:1.5em; color:yellow; margin:15px; font-weight:bold;">IDLE</div>
  <div class="compass"><div class="needle target" id="t"></div><div class="needle robot" id="r"></div></div>
  
  <div class="card">
    <span id="dL" style="margin:0 10px;">L: 0</span> | 
    <span id="dF" style="margin:0 10px;">F: 0</span> | 
    <span id="dR" style="margin:0 10px;">R: 0</span>
  </div>
  
  <div class="card">
    <strong>YAW:</strong> <span id="y">0</span>&deg; | 
    <strong>DIST:</strong> <span id="d">0</span>m | 
    <strong>WP:</strong> <span id="wp">0</span>/11<br>
    <strong>IMU CAL:</strong> <span id="m">0</span>/3 | 
    <strong>GPS:</strong> <span id="g">WAIT</span>
    <span id="overrideIndicator" style="display:none; color:#ff5722; font-weight:bold;"> | OVERRIDE</span><span id="hwOverrideIndicator" style="display:none; color:#ff5722; font-weight:bold;"> | HW OVERRIDE</span>
  </div>

  ...
  
  <div style="margin-top: 20px;">
    <button class="btn-start" id="startNavBtn" onclick="startNav()" enabled>START MISSION</button>
    <button class="btn-home" id="homeBtn" onclick="returnHome()" enabled> RETURN TO HOME</button>
    <button class="btn-stop" onclick="emergencyStop()">EMERGENCY STOP</button>
  </div>
  
  <div class="card" style="margin-top: 15px;">
    <h3 style="margin: 5px 0;">Data Logging</h3>
    <button onclick="fetch('/startLog')" style="background:#28a745; color:white; width:45%;">START LOG</button>
    <button onclick="fetch('/stopLog')" style="background:#dc3545; color:white; width:45%;">STOP LOG</button><br>
    <button onclick="window.location.href='/download'" style="background:#007bff; color:white; width:92%; margin-top:8px;">DOWNLOAD CSV</button>
  </div>
  
  <script>
    let imuReady = false;
    let hwChecked = false;
    let expModOK = false;
    let dacOK = false;
    let checkupDone = false;
    let isOverride = false;
    let isHwOverride = false;
    
    function initIMU() {
      document.getElementById('imuBtn').disabled = true;
      document.getElementById('imuBtn').innerText = 'Initializing IMU...';
      document.getElementById('imuSection').innerText = 'Initializing BNO055...';
      document.getElementById('imuSection').className = 'status-box status-progress';
      
      fetch('/initIMU').then(r => r.text()).then(status => {
        if(status === 'OK') {
          imuReady = true;
          document.getElementById('imuDetails').style.display = 'block';
          document.getElementById('imuSection').innerText = 'Calibrating Magnetometer... (Move in figure-8)';
          document.getElementById('imuSection').className = 'status-box status-progress';
          document.getElementById('imuBtn').style.display = 'none';
          
          // Show next step
          document.getElementById('hwSection').style.display = 'block';
          document.getElementById('hwCheckBtn').style.display = 'block';
        } else {
          alert('IMU initialization failed! Check I2C connections.');
          document.getElementById('imuBtn').disabled = false;
          document.getElementById('imuBtn').innerText = 'RETRY IMU INIT';
          document.getElementById('imuSection').innerText = 'IMU Init Failed';
          document.getElementById('imuSection').className = 'status-box status-error';
        }
      });
    }
    
    function checkHardware() {
      document.getElementById('hwCheckBtn').disabled = true;
      document.getElementById('hwCheckBtn').innerText = ' Checking Hardware...';
      document.getElementById('hwSection').innerText = ' Scanning I2C devices...';
      document.getElementById('hwSection').className = 'status-box status-progress';
      
      fetch('/checkHardware').then(r => r.json()).then(data => {
        hwChecked = true;
        expModOK = data.expMod || data.hwOverride;
        dacOK = data.dac || data.hwOverride;
        isHwOverride = !!data.hwOverride;
        
        document.getElementById('hwDetails').style.display = 'block';
        document.getElementById('expModStatus').innerText = 'MCP23017: ' + (expModOK ? (isHwOverride ? ' OVERRIDDEN' : 'Yes') : 'No');
        document.getElementById('expModStatus').style.background = expModOK ? (isHwOverride ? '#ff5722' : '#4caf50') : '#f44336';
        document.getElementById('dacStatus').innerText = 'MCP4728: ' + (dacOK ? (isHwOverride ? ' OVERRIDDEN' : 'Yes') : 'No');
        document.getElementById('dacStatus').style.background = dacOK ? (isHwOverride ? '#ff5722' : '#4caf50') : '#f44336';
        
        if (expModOK && dacOK) {
          if (isHwOverride) {
            document.getElementById('hwSection').innerText = '2 Hardware Ready (OVERRIDE)';
            document.getElementById('hwSection').className = 'status-box status-override';
            document.getElementById('hwOverrideIndicator').style.display = 'inline';
          } else {
            document.getElementById('hwSection').innerText = '2 Yes Hardware Ready';
            document.getElementById('hwSection').className = 'status-box status-ready';
          }
          document.getElementById('hwCheckBtn').style.display = 'none';
          
          // Show next step
          document.getElementById('checkupSection').style.display = 'block';
          document.getElementById('checkupBtn').style.display = 'block';
          document.getElementById('skipCheckupBtn').style.display = 'block';
        } else {
          document.getElementById('hwSection').innerText = '2 No Hardware Error';
          document.getElementById('hwSection').className = 'status-box status-error';
          document.getElementById('hwCheckBtn').disabled = false;
          document.getElementById('hwCheckBtn').innerText = 'RETRY HARDWARE CHECK';
        }
      });
    }
    
    function runCheckup() {
      document.getElementById('checkupBtn').disabled = true;
      document.getElementById('checkupBtn').innerText = 'Running Motor Checkup...';
      document.getElementById('checkupSection').innerText = 'Testing all motors...';
      document.getElementById('checkupSection').className = 'status-box status-progress';
      
      fetch('/runCheckup').then(r => r.text()).then(status => {
        if(status === 'OK' || status === 'ALREADY_DONE') {
          checkupDone = true;
          document.getElementById('checkupSection').innerText = '3 Yes Motor Checkup Complete';
          document.getElementById('checkupSection').className = 'status-box status-ready';
          document.getElementById('checkupBtn').style.display = 'none';
          document.getElementById('skipCheckupBtn').style.display = 'none';
          
          updateNavButtons();
        }
      }).catch(e => {
        alert('Motor checkup failed!');
        document.getElementById('checkupBtn').disabled = false;
        document.getElementById('checkupBtn').innerText = ' STEP 3: RUN MOTOR CHECKUP';
        document.getElementById('checkupSection').innerText = '3 Yes Checkup Ready';
        document.getElementById('checkupSection').className = 'status-box status-error';
        // Ensure skip button is still available if user wants to skip
        document.getElementById('skipCheckupBtn').disabled = false;
        document.getElementById('skipCheckupBtn').innerText = 'SKIP MOTOR CHECKUP & ENABLE START';
      });
    }

    function updateNavButtons() {
      let calibrationReady = (document.getElementById('m').innerText === '3') || isOverride;
      let gpsReady = document.getElementById('g').innerText === 'FIXED';
      let hardwareReady = (expModOK && dacOK) || isHwOverride;
      
      if (checkupDone && calibrationReady && gpsReady && hardwareReady) {
        document.getElementById('startNavBtn').disabled = false;
        document.getElementById('homeBtn').disabled = false;
      }
    }
    
    function startNav() {
      fetch('/startNav').then(r => {
        if (r.status === 403) {
          return r.text().then(msg => alert('Cannot start: ' + msg));
        }
      });
    }
    
    function returnHome() {
      if(confirm('Return to home waypoint (first position)?')) {
        fetch('/returnHome').then(r => {
          if (r.status === 200) {
            alert('Navigating to home position!');
          } else {
            return r.text().then(msg => alert('Cannot return home: ' + msg));
          }
        });
      }
    }
    
    function emergencyStop() {
      fetch('/emergencyStop');
    }
    
    setInterval(() => {
      fetch('/data').then(r => r.json()).then(d => {
        document.getElementById('t').style.transform = `rotate(${d.b}deg)`;
        document.getElementById('r').style.transform = `rotate(${d.y}deg)`;
        document.getElementById('cmd').innerText = d.c;
        document.getElementById('y').innerText = d.y.toFixed(1);
        document.getElementById('d').innerText = d.dist.toFixed(1);
        document.getElementById('wp').innerText = d.wp;
        document.getElementById('m').innerText = d.m;
        document.getElementById('magCal').innerText = d.m + '/3';
        document.getElementById('g').innerText = d.g ? "FIXED" : "WAIT";
        
        const s = {dL: d.dL, dF: d.dF, dR: d.dR};
        for (let k in s) {
          let e = document.getElementById(k);
          e.innerText = k.substring(1) + ": " + s[k].toFixed(0);
        }
        
        if(d.imuInit && !imuReady) {
          imuReady = true;
        }
        
        if(d.override) {
          isOverride = true;
          document.getElementById('overrideIndicator').style.display = 'inline';
        }
        
        if(d.m === 3 && imuReady && !isOverride && document.getElementById('imuSection').className.includes('progress')) {
          document.getElementById('imuSection').innerText = '1 Yes IMU Ready & Calibrated';
          document.getElementById('imuSection').className = 'status-box status-ready';
        }
        
        
        if(d.checkup && !checkupDone) {
          checkupDone = true;
          document.getElementById('checkupSection').innerText = '3 Yes Motor Checkup Complete';
          document.getElementById('checkupSection').className = 'status-box status-ready';
          document.getElementById('checkupBtn').style.display = 'none';
          document.getElementById('skipCheckupBtn').style.display = 'none';
        }
        
        updateNavButtons();
      });
    }, 200);
  </script>
</body></html>)rawliteral";

// Wrapper to call Controller.h function and update local tracker
void cyclePosture() {
  Serial.print("Cycling Posture... Current State: "); Serial.print(postureState);
  // lockLaydownStand(); // Calls the function from Controller.h
  
  postureState++;
  if (postureState > 2) postureState = 0; // Reset to 0 (Stand) after 2 (Lay)
  
  Serial.print(" -> New State: "); Serial.println(postureState);
}

void ensureStanding() {
  Serial.println("Checking Posture: MUST STAND (Target: 0)");
  // If Locked (1) -> Cycle twice to Stand
  // If Laying (2) -> Cycle once to Stand
  while (postureState != 0) {
    // cyclePosture();
    delay(2500); // Wait for physical movement
  }
  Serial.println("Posture Verified: STANDING");
}

void ensureLayingDown() {
  Serial.println("Checking Posture: MUST LAY DOWN (Target: 2)");
  // If Standing (0) -> Cycle twice (Lock->Lay)
  // If Locked (1)   -> Cycle once (Lay)
  while (postureState != 2) {
    // cyclePosture();
    delay(2500); // Wait for physical movement
  }
  Serial.println("Posture Verified: LAYING DOWN");
}

float angleDiff(float target, float current) {
  float diff = target - current;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

void handleDataLogging() {
  // 1. Check if logging is active and if enough time has passed
  if (!loggingActive || (millis() - lastLogTime < LOG_INTERVAL_MS)) {
    return; 
  }

  lastLogTime = millis(); // Reset timer

  // 2. Open file in APPEND mode
  File file = SPIFFS.open(csvPath, FILE_APPEND); 
  
  if (file) {
    // 3. Create a comma-separated string (CSV format)
    // REQUESTED FORMAT: Phase, Longitude, Latitude
    String dataLine = String(currentPhase) + "," + 
                      String(curLon, 6) + "," + 
                      String(curLat, 6) + "\n";
                      
    file.print(dataLine);
    file.close(); // Close immediately to save data
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Rover Startup ===");
  
  Wire.begin();
  Serial.println("I2C bus initialized");
  
  if(display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED display initialized");
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.print("ROVER BOOTING...");
    display.display();
  } else {
    Serial.println("OLED init failed");
  }
  
  Serial.println("IMU initialization deferred (manual via web)");
  
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  configureGPS();
  Serial.println("GPS configured");
  
  SPIFFS.begin(true);
  Serial.println("SPIFFS mounted");
  
  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG, OUTPUT); pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT); pinMode(RIGHT_ECHO, INPUT);
  pinMode(BACK_TRIG, OUTPUT); pinMode(BACK_ECHO, INPUT);
  Serial.println("Ultrasonic sensors configured");

  Serial.print("Setting up WiFi AP... ");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("WiFi AP ready!");
  Serial.print("AP IP address: ");
  Serial.println(IP);  // Usually 192.168.4.1

  server.on("/", [](){ server.send_P(200, "text/html", webpage); });

  server.on("/data", [](){
    String j = "{\"y\":"+String(trueYaw)+",\"b\":"+String(bearing)+",\"dist\":"+String(distance)+
               ",\"m\":"+String(m_cal)+",\"wp\":"+String(wpIdx+1)+",\"c\":\""+String(cmdNames[currentCmd])+"\"" +
               ",\"dF\":"+String(distF)+",\"dL\":"+String(distL)+",\"dR\":"+String(distR)+
               ",\"l\":"+(loggingActive?1:0)+",\"g\":"+(gps.location.isValid()?1:0)+
               ",\"imuInit\":"+(imuInitialized?1:0)+
               ",\"override\":"+(calibrationOverride?1:0)+
               ",\"hwOverride\":"+(hardwareOverride?1:0)+
               ",\"checkup\":"+(checkupCompleted?1:0)+
               ",\"hwChecked\":"+(hardwareChecked?1:0)+
               ",\"expMod\":"+(expModReady?1:0)+
               ",\"dac\":"+(dacReady?1:0)+"}";
    server.send(200, "application/json", j);
  });

  server.on("/initIMU", [](){
    if (!imuInitialized) {
      Serial.println("\n=== Initializing BNO055 IMU ===");
      if(bno.begin()) {
        imuInitialized = true;
        Serial.println("BNO055 initialized successfully!");
        Serial.println("Please calibrate magnetometer (move in figure-8 pattern)");
        server.send(200, "text/plain", "OK");
      } else {
        Serial.println("ERROR: BNO055 initialization failed!");
        server.send(500, "text/plain", "FAILED");
      }
    } else {
      server.send(200, "text/plain", "ALREADY_INIT");
    }
  });

  server.on("/overrideCalibration", [](){
    calibrationOverride = true;
    imuCalibrated = true;
    Serial.println(" WARNING: IMU CALIBRATION OVERRIDDEN!");
    Serial.println("Navigation enabled without full magnetometer calibration.");
    Serial.println("Heading accuracy may be significantly degraded!");
    server.send(200, "text/plain", "OK");
  });

  server.on("/overrideHardware", [](){
    hardwareOverride = true;
    expModReady = true;
    dacReady = true;
    hardwareChecked = true;
    Serial.println(" WARNING: HARDWARE CHECK OVERRIDDEN! Forcing DAC & I/O Expander as READY.");
    server.send(200, "text/plain", "OK");
  });

  server.on("/clearHardwareOverride", [](){
    hardwareOverride = false;
    expModReady = false;
    dacReady = false;
    hardwareChecked = false;
    Serial.println("Hardware override CLEARED.");
    server.send(200, "text/plain", "OK");
  });

  server.on("/checkHardware", [](){
    if (!imuInitialized && !calibrationOverride) {
      server.send(403, "text/plain", "Initialize IMU first or use override");
      return;
    }
    
    Serial.println("\n=== Checking Hardware (DAC & I/O Expander) ===");
    if (!hardwareOverride) {
      // Normal flow: probe and initialize external modules
      expModReady = initExpMod();
      dacReady = initDAC();
    } else {
      // Override flow: do NOT touch hardware, just simulate ready state
      expModReady = true;
      dacReady = true;
      Serial.println(" Hardware override active - skipping initExpMod/initDAC and marking devices READY.");
      // Update OLED briefly to show simulated status
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("HW OVERRIDE: SKIPPED PROBE");
      display.setCursor(0,10);
      display.print("DAC/I2C simulated READY");
      display.display();
    }
    hardwareChecked = true;

    String response = "{\"expMod\":";
    response += expModReady ? "true" : "false";
    response += ",\"dac\":";
    response += dacReady ? "true" : "false";
    response += ",\"hwOverride\":";
    response += hardwareOverride ? "true" : "false";
    response += "}";

    Serial.print("  MCP23017 (I/O Expander): ");
    Serial.println(expModReady ? "Yes READY" : "No NOT FOUND");
    Serial.print("  MCP4728 (DAC): ");
    Serial.println(dacReady ? "Yes READY" : "No NOT FOUND");

    server.send(200, "application/json", response);
  });

  server.on("/runCheckup", [](){
    if (!imuInitialized && !calibrationOverride) {
      server.send(403, "text/plain", "Initialize IMU first or use override");
      return;
    }
    
    if (!(hardwareChecked || hardwareOverride)) {
      server.send(403, "text/plain", "Run hardware check first or use hardware override");
      return;
    }

    if (!expModReady || !dacReady) {
      if (!hardwareOverride) {
        server.send(403, "text/plain", "Hardware not ready");
        return;
      } else {
        Serial.println(" Hardware override active - simulating motor checkup.");
      }
    }

    if (!checkupCompleted) {
      if (!hardwareOverride) {
        Serial.println("\n=== Starting Motor Checkup Routine ===");
        initialcheckuproutine();
        Serial.println("=== Motor Checkup Complete ===\n");
      } else {
        // DO NOT call Controller functions when overridden; just simulate
        Serial.println(" Motor checkup SIMULATED due to hardware override.");
        // Visual feedback on OLED
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0,0);
        display.print("Motor Checkup:");
        display.setCursor(0,10);
        display.print("SIMULATED (OVERRIDE)");
        display.display();
      }
      checkupCompleted = true;
      server.send(200, "text/plain", "OK");
    } else {
      server.send(200, "text/plain", "ALREADY_DONE");
    }
  });

  // Skip the motor checkup and mark it complete (if hardware is OK or override active)
  server.on("/skipCheckup", [](){
    if (!imuInitialized && !calibrationOverride) {
      server.send(403, "text/plain", "Initialize IMU first or use override");
      return;
    }

    if (!(hardwareChecked || hardwareOverride)) {
      server.send(403, "text/plain", "Run hardware check first or use hardware override");
      return;
    }

    checkupCompleted = true;
    Serial.println(" Motor checkup SKIPPED by user");
    server.send(200, "text/plain", "OK");
  });

  server.on("/startNav", [](){
    if (!((hardwareChecked && expModReady && dacReady) || hardwareOverride)) {
      server.send(403, "text/plain", "Hardware check required");
      return;
    }
    if (!checkupCompleted) {
      server.send(403, "text/plain", "Motor checkup required");
      return;
    }
    if (m_cal != 3 && !calibrationOverride) {
      server.send(403, "text/plain", "IMU calibration incomplete (or use override)");
      return;
    }
    if (!gps.location.isValid()) {
      server.send(403, "text/plain", "GPS fix required");
      return;
    }

    returnHomeMode = false;
    navStarted = true;
    wpIdx = 0;
    currentPhase = HIT_D_BUILD;
    Serial.println(">>> NAVIGATION STARTED <<<");
    if(calibrationOverride) {
      Serial.println(" Running in OVERRIDE mode - heading may be inaccurate!");
    }
    if(hardwareOverride) {
      Serial.println(" Running with HARDWARE OVERRIDE - controller functions were NOT executed.");
    }
    server.send(200);
  });

  server.on("/returnHome", [](){
    if (!((hardwareChecked && expModReady && dacReady) || hardwareOverride)) {
      server.send(403, "text/plain", "Hardware check required");
      return;
    }
    if (!checkupCompleted) {
      server.send(403, "text/plain", "Motor checkup required");
      return;
    }
    if (m_cal != 3 && !calibrationOverride) {
      server.send(403, "text/plain", "IMU calibration incomplete (or use override)");
      return;
    }
    if (!gps.location.isValid()) {
      server.send(403, "text/plain", "GPS fix required");
      return;
    }

    returnHomeMode = true;
    navStarted = true;
    wpIdx = 0;
    currentPhase = HIT_D_BUILD;
    Serial.println(">>> RETURN TO HOME INITIATED <<<");
    if(calibrationOverride) {
      Serial.println(" Running in OVERRIDE mode - heading may be inaccurate!");
    }
    if(hardwareOverride) {
      Serial.println(" Running with HARDWARE OVERRIDE - controller functions were NOT executed.");
    }
    server.send(200);
  });

  server.on("/startLog", [](){
    loggingActive = true;
    File f = SPIFFS.open(csvPath, "w");
    // FIXED HEADER: Matches Phase, Lon, Lat
    f.println("Phase,Lon,Lat"); 
    f.close();
    Serial.println("Data logging started");
    server.send(200);
  });

  server.on("/stopLog", [](){ 
    loggingActive = false; 
    Serial.println("Data logging stopped");
    server.send(200); 
  });
  
  server.on("/download", [](){ 
    File f = SPIFFS.open(csvPath, "r"); 
    server.streamFile(f, "text/csv"); 
    f.close(); 
  });

  server.on("/emergencyStop", [](){
    // stop();
    navStarted = false;
    currentCmd = CMD_IDLE;
    Serial.println(">>> EMERGENCY STOP <<<");
    server.send(200);
  });

  server.begin();
  Serial.println("\n=== WEB SERVER RUNNING ===");
  Serial.println("Access rover at: http://" + WiFi.localIP().toString());
  Serial.println("\nInitialization Sequence:");
  Serial.println("  1. Initialize IMU via web interface (or use OVERRIDE)");
  Serial.println("  2. Check DAC & I/O Expander");
  Serial.println("  3. Run motor checkup");
  Serial.println("  4. Wait for GPS fix and IMU calibration (or use OVERRIDE)");
  Serial.println("  5. Start navigation or return home\n");
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("WEB SERVER READY");
  display.setCursor(0, 15);
  display.print("IP: ");
  display.setCursor(0, 25);
  display.print(WiFi.localIP().toString());
  display.setCursor(0, 45);
  display.print("Use web interface");
  display.display();
}

void loop() {
  server.handleClient();

  if (imuInitialized) {
    if (!imuCalibrated && !calibrationOverride) {
      uint8_t s, g, a;
      bno.getCalibration(&s, &g, &a, &m_cal);
      if (m_cal == 3) {
        imuCalibrated = true;
        Serial.println(">>> IMU CALIBRATION COMPLETE <<<");
      }
    }

    sensors_event_t ev;
    bno.getEvent(&ev, Adafruit_BNO055::VECTOR_EULER);
    trueYaw = ev.orientation.x + TOTAL_NORTH_OFFSET;
    if (trueYaw >= 360) trueYaw -= 360;
    if (trueYaw < 0) trueYaw += 360;
  }

  static unsigned long lastNav = 0;
  if (millis() - lastNav > 50) {
    lastNav = millis();
    updateNavigation();
    executeMotorCommand();
    handleDataLogging();
  }

  while (GPS_Serial.available() > 0) {
    if (gps.encode(GPS_Serial.read())) {
      curLat = gps.location.lat();
      curLon = gps.location.lng();
    }
  }

  static unsigned long lastOLED = 0;
  if (millis() - lastOLED > 200) {
    lastOLED = millis();
    display.clearDisplay();
    display.setTextColor(WHITE);

    if (!imuInitialized && !calibrationOverride) {
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.print("IMU INIT");
      display.setTextSize(2);
      display.setCursor(0, 30);
      display.print("REQUIRED");
    } else if (!hardwareChecked) {
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.print("HW CHECK");
      display.setTextSize(2);
      display.setCursor(0, 30);
      display.print("REQUIRED");
    } else if (!expModReady || !dacReady) {
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.print("HW ERROR");
      display.setTextSize(2);
      display.setCursor(0, 30);
      display.print("CHECK I2C");
    } else if (!checkupCompleted) {
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.print("MOTOR CHECKUP");
      display.setTextSize(2);
      display.setCursor(0, 30);
      display.print("PENDING");
    } else if (!imuCalibrated && !calibrationOverride) {
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.print("CALIBRATING IMU");
      display.setTextSize(2);
      display.setCursor(0, 30);
      display.printf("MAG: %d/3", m_cal);
    } else if (!gps.location.isValid()) {
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.print("WAITING GPS FIX");
      display.setTextSize(2);
      display.setCursor(0, 30);
      display.print("NO FIX");
      if(calibrationOverride) {
        display.setTextSize(1);
        display.setCursor(0, 55);
        display.print("OVERRIDE MODE");
      }
    } else if (currentPhase == END_MISSION) {
      if (missionended)
      {
      display.setTextSize(2);
      display.setCursor(15, 25);
      display.print("REACHED!");
      }
      else
      {
        display.setTextSize(2);
        display.setCursor(15, 25);
        display.print(cmdNames[currentCmd]);
      }
    } else if (currentPhase == ORIENT_NORTH) {
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.print("ORIENTING");
      display.setTextSize(2);
      display.setCursor(0, 30);
      display.printf("YAW:%03.0f", trueYaw);
      display.setTextSize(1);
      display.setCursor(0, 55);
      display.print("TARGET: NORTH");
    } else {
      display.setTextSize(2);
      display.setCursor(0, 0);
      // display.printf("YAW:%03.0f", trueYaw);
      if(calibrationOverride) {
        display.setTextSize(1);
        display.setCursor(100, 2);
        display.print("OVR");
      }
      display.drawLine(0, 20, 128, 20, WHITE);
      display.setTextSize(2);
      display.setCursor(15, 25);
      display.print(cmdNames[currentCmd]);
      // display.setCursor(0, 40);
      // display.printf("WP:%d/%d D:%.1fm", wpIdx+1, NUM_WP, distance);
    }
    display.display();
  }
}
