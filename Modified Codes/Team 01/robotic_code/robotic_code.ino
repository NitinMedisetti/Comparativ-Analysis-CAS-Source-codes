
//Tested on 21 Jan, successfully reached H but did not laydown

// This is the release verison of the code. It was frozen at the 2026_01_19_ at 9:15 pm
// and thus all Changes must be communicated with the whole team bevore implementing. The basis of this code was tested succesfully several times
// last adjustments included deleting unused variables, clean up of comments and included a improved verison of the webHMI.

// This code was succesfully verified with the roboDog at 2026_01_14_ at 6 15 pm reaching Bullseye at H. Manuall direction for the IMU was used
// Rules for Programming in this sketch:
//Maintain the programs structure do not use auto format or an AI Tool for the whole sketch
// if defining konstants then start the name with the small letter "k"
// e.g. k_LidarMountingAngleRegardingFront
// keep naming of FSM dependent States, variables and constants consistent i.e. State dependent declarations start with either "OS","P","H" dependent on the navigation phase they are used in (use OS=Open Sea; P=Navigation to Post Office; H=Navigation to H-building)
// e.g. kOS_Example
// e.g. kP_Example
// e.g. kH_Example
// Bigger classes are to be defined in seperate .h and .cpp files. Additionally develop both files in such a way, that the main sketch is as short and simple as possible.
// declarating variables in the loop is strictly forbidden. Only create them in the respective code section (Constants, Variables).
// Everything that you do needs to be able to be copied and put into the final sketch. So keep that in mind when coding.
// Use the output class with the object "out" for all outputs to the robot
// Do not touch the setup.
// Do not change existing variable, constant, object names.
// Do not touch any .cpp or .h file
// Do not add a .cpp or .h file -> every .cpp and .h file that needs to be added is first requested, then validated and only then included by Jonas
// Do not change the output class
// Do not change sensor reading funcitons even if they are wrong. If this causes problems comment them out.
// The delay funciton is forbidden to use
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include "Controller.h"
#include "DisplayManager.h"
#include "ImuManager.h"
#include "UsManager.h"
#include "GpsManager.h"
#include "WebHMI.h"
#include "LidarAlign.h"
// -----------------------------------------------------------------------------
// Output-class
class Output {
public:
  Output(DisplayManager& d, WebHMI& w)
    : _disp(d), _hmi(w) {}

  // pro loop() einmal aufrufen, dann ist genau 1 Output-Befehl erlaubt
  void beginCycle() {
    _sentThisCycle = false;
  }

  // --- Bewegung ---
  void stop() {
    _sendNoValue("Stop", "Stop", []() {
      ::stop();
    });
  }

  void forward(int pct) {
    _sendWithPct("Forward", "Forward", pct, [](int v) {
      ::forward(v);
    });
  }
  void backward(int pct) {
    _sendWithPct("Backward", "Backward", pct, [](int v) {
      ::backward(v);
    });
  }

  void goLeft(int pct) {
    _sendWithPct("GoLeft", "GoLeft", pct, [](int v) {
      ::stepLeft(v);
    });
  }
  void goRight(int pct) {
    _sendWithPct("GoRight", "GoRight", pct, [](int v) {
      ::stepRight(v);
    });
  }

  void turnLeft(int pct) {
    _sendWithPct("RotateLeft", "RotateLeft", pct, [](int v) {
      ::rotateLeft(v);
    });
  }
  void turnRight(int pct) {
    _sendWithPct("RotateRight", "RotateRight", pct, [](int v) {
      ::rotateRight(v);
    });
  }

  // --- Actions (blockieren ~1s; nur in passenden States nutzen) ---
  void clickStart() {
    _sendNoValue("ClickStart", "ClickStart", []() {
      ::clickStart();
    });
  }
  void lockLaydownStand() {
    _sendNoValue("LockLaydownStand", "LockLayStand", []() {
      // 2026_01_17_ no call due to unique usage
    });
  }

private:
  // Limit the Display to 20 characters
  static constexpr size_t kDispMax = 20;

  DisplayManager& _disp;
  WebHMI& _hmi;

  bool _sentThisCycle = false;

  // last UI state
  String _lastWeb = "";
  String _lastDispLine2 = "";

  bool _guardOneCommand() {
    if (_sentThisCycle) {
      // ignore second command in same loop.
      _publishUi("IGNORED", "IGNORED", "IGNORED (2 cmds)");
      return false;
    }
    _sentThisCycle = true;
    return true;
  }

  static int _clampPct(int pct) {
    // not used anymore, since controller's max is capt at 28 from Controller.h
    return constrain(pct, 0, SPEED_LIMIT);
  }

  void _publishUi(const char* webCmd, const char* dispCmd, const char* webTextOverride = nullptr) {
    // ---- Display ----
    char d2[kDispMax + 1];
    _snprintfTrunc(d2, sizeof(d2), "%s", dispCmd);

    if (_lastDispLine2 != String(d2)) {
      _disp.show("Out", d2);
      _lastDispLine2 = String(d2);
    }

    // ---- WebHMI ----
    String webText = (webTextOverride != nullptr) ? String(webTextOverride) : String(webCmd);
    if (_lastWeb != webText) {
      _hmi.showValue("Output", webText);
      _lastWeb = webText;
    }
  }

  template<typename Fn>
  void _sendNoValue(const char* webCmd, const char* dispCmd, Fn fn) {
    if (!_guardOneCommand()) return;

    fn();

    _publishUi(webCmd, dispCmd);
  }

  template<typename Fn>
  void _sendWithPct(const char* webCmd, const char* dispCmd, int pct, Fn fn) {
    if (!_guardOneCommand()) return;

    const int v = _clampPct(pct);
    fn(v);

    // Display: "Cmd 25"
    char d2[kDispMax + 1];
    _snprintfTrunc(d2, sizeof(d2), "%s %d", dispCmd, v);

    // Web: "Cmd 25"
    String webText = String(webCmd) + " " + String(v);


    if (_lastDispLine2 != String(d2)) {
      _disp.show("Out", d2);
      _lastDispLine2 = String(d2);
    }
    if (_lastWeb != webText) {
      _hmi.showValue("Output", webText);
      _lastWeb = webText;
    }
  }

  static void _snprintfTrunc(char* dst, size_t dstSize, const char* fmt, ...) {
    if (!dst || dstSize == 0) return;
    va_list args;
    va_start(args, fmt);
    vsnprintf(dst, dstSize, fmt, args);
    va_end(args);
    dst[dstSize - 1] = '\0';
  }
};
// -----------------------------------------------------------------------------
// Object creation
DisplayManager disp;
WebHMI hmi;
ImuManager imu;
UsManager us;
GpsManager gps;
Output out(disp, hmi);
//

//Constants
// Only add constants in the correct section for the respective FSM
// Wall distances
// OS
int kOS_DistCmFrontStopPost = 100;     // Robot walks from GPS WP to post at a fixed angle at this distance from the wall it should stop
int kOS_DistCmRightStepPostWall = 50;  // Robot steps towards the parallel Wall of the Post office on the right side until it reaches this distance
int kOS_DistCmRigthWallWalkPost = 50;  // The Robot walks towards the Final Point at the Post office, this is the distance it keeps from the right side of the wall
int kOS_DistCmFrontStopPostDor = 100;  // The Robot stops at a distance to the door-Wall of the Post building and thus is at its final position
                                       // To Post
                                       // To H-Building

// Wall distance Toleraces
// OS
int kOS_TolDistCmFrontStopPost = 30;      //
int kOS_TolDistCmRightStepPostWall = 30;  //
int kOS_TolDistCmRigthWallWalkPost = 30;  //
int kOS_TolDistCmFrontStopPostDor = 30;   //
                                          // To Post
                                          // To H-Building

// Constant Yaw Values in respect to the local coordinate system convention (left is positiv, right negativ)
// OS
int kOS_AngleDegToPost = -150;             // The robot is at the GPS waypoint before the Post and rotates to the Coordinate systems Angle of
int kOS_AngleDegParallelToPostWall = -90;  // The robot recognised the wall of the Post building and Rotates to be parallel with it
int kOS_AngleDegStartDir = 90;             // The Robot rotates to its position where it should face to start the walk to H
                                           // To Post
                                           // To H-Building

// Yaw Tolerances for fixed Turns
// OS
int kOS_TolAngleDegToPost = 5;
int kOS_TolAngleDegParallelToPostWall = 5;
int kOS_TolAngleDegStartDir = 5;
// To Post
// To H-Building

// Waypoints (WP)
// OS
float kOS_xWPSafepoint = 9999;     // The Waypoint we go to after we wake up in the open see
float kOS_yWPSafepoint = 9999;     // The Waypoint we go to after we wake up in the open see
float kOS_xWPRightToBench = 9999;  // WP on the right side of the most right bench when standing in front of Post Building (next to a trash can)
float kOS_yWPRightToBench = 9999;  // WP on the right side of the most right bench when standing in front of Post Building (next to a trash can)
                                   // To Post
                                   // To H-Building

// WP tolerances for GPS
// OS
int kOS_TolWpCmSafepoint = 100;     // WP Tolerance in meter
int kOS_TolWpCmRightToBench = 100;  // WP Tolerance in meter
                                    // To Post
                                    // To H-Building

// Standard Robot Speeds
int k_LowSpeed = 5;        // next to buildings or in risky situations
int k_StandardSpeed = 20;  // normal traveling speed
int k_HighSpeed = 28;      // for low risk sections

// Lidar Mounting angle
int k_LidarMountingAngleRegardingFront = 90;  // the 0 point of the lidar is 90 degree right to the Kits front
                                              //
                                              //
                                              // Variables
// Variables for Sensor readings
// GPS
float xCurrent = 0.0f;       // xCurrent      = gps.getX();
float yCurrent = 0.0f;       // yCurrent      = gps.getY();
                             // IMU
float currentYawDeg = 0.0f;  // currentYawDeg = imu.getCampusYawDeg();
float startYawDeg = 0.0f;
float currentPitchDeg = 0.0f;
float currentRollDeg = 0.0f;
// Ultrasonic
float usFrontM = 99.0f;   // usFrontM      = us.getFrontMeters();
float usAngeled = 99.0f;  // usRightM      = us.getRightMeters();
float usRearM = 99.0f;    // usRightM      = us.getRightMeters();
float usRightM = 99.0f;   // usRightM      = us.getRightMeters();
float usLeftM = 99.0f;    // usRightM      = us.getRightMeters();
// Lidar
int rxPin = 27;
int txPin = 33;
LidarDetectResult lidarResult;
//fsm timer
float lastFsmUpdate = 0;
// required indicators for navigation and buttons
bool pth_navStart = false;

// helper functions variables
float pth_YawErr = 0;
String fsmstate = "nil";
// For button output
static unsigned long lastHmiUpdate = 0;
static String uiStateText;                                //2026_01_17_added
static const char* kWaitForCommand = "wait for command";  //2026_01_17_added
//--------------------------

//Helper functions
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

bool isYawClose(float current, float target, float tolerance = 3.0) {
  current = normalizeAngle(current);
  target = normalizeAngle(target);
  float diff = normalizeAngle(target - current);
  pth_YawErr = diff;
  return abs(diff) <= tolerance;
}

void commandSet(String command) {
  if (command == "forward") out.forward(k_HighSpeed);
  if (command == "forward slow") out.forward(k_StandardSpeed);
  if (command == "stop" || command == "no wall") out.stop();
  if (command == "step left") out.goLeft(k_HighSpeed);
  if (command == "step right") out.goRight(k_HighSpeed);
  if (command == "rotate left") out.turnLeft(k_StandardSpeed);
  if (command == "rotate right") out.turnRight(k_StandardSpeed);
  if (command == "step back") out.backward(k_StandardSpeed);
}
//-------------------------

// States for ultrasonic helper class and constants
enum WallFollowState {
  WF_IDLE,
  WF_MEASURE,
  WF_EVALUATE
};
//--------------------------

// Current state
WallFollowState wfState = WF_IDLE;
//--------------------------

// Constants used for ultrasonic helper class
int wfSide = 1;
int wfTargetDist = 150;
long wfCurrentDist = 0;
String wfCommand = "STOP";
//--------------------------

// ultrasonic helper class for wall following
void wallFollowingFSM(int side, int targetDistance, int kwfTolerance = 30) {  //side = ( 1 - left, 2 - right )
  // Update parameters
  wfSide = side;
  wfTargetDist = targetDistance;
  if (wfTargetDist > 280) wfTargetDist = 280;
  if (wfTargetDist < 30) wfTargetDist = 40;

  switch (wfState) {
    case WF_IDLE:
      wfState = WF_MEASURE;
      fsmstate = "wf_idle";
      break;

    case WF_MEASURE:
      wfCurrentDist = (wfSide == 1) ? usLeftM : usRightM;
      wfState = WF_EVALUATE;
      fsmstate = "wf_measure";
      break;

    case WF_EVALUATE:
      if (wfCurrentDist >= 300) {
        wfCommand = "no wall";
      } else {
        int error = wfCurrentDist - wfTargetDist;
        if (abs(error) <= kwfTolerance) {
          wfCommand = "forward";
        } else if (error < -kwfTolerance) {
          wfCommand = (wfSide == 1) ? "step right" : "step left";
        } else {
          wfCommand = (wfSide == 1) ? "step left" : "step right";
        }
      }
      wfState = WF_MEASURE;
      fsmstate = "wf_evalute";
      break;
  }
}
//--------------------------

// States for post to h
enum Post_to_H {
  PTH_IDLE,
  PTH_STAND_UP,
  PTH_TO_D,
  PTH_D_TO_CANAL,
  PTH_THROUGH_CANAL,
  PTH_CANAL_TO_K,
  PTH_K_TO_H,
  PTH_H_TO_POLE,
  PTH_LAY_DOWN
};
//--------------------------

//Current State of fsm
Post_to_H pthState = PTH_IDLE;
//--------------------------

//FSM constants
String pthCommand = "stop";
bool pth_obstacle_override = false;
bool pthRotated = false;
bool pthRotated2 = false;
bool ptharrived = false;
float pthRotationTarget = 0;
//--------------------------

//navigation fsm
void POST_TO_H_NAV_FSM() {
  switch (pthState) {
    case PTH_IDLE:
      pthState = PTH_STAND_UP;
      fsmstate = "pth_idle";
      break;

    case PTH_STAND_UP:
      pthCommand = "stand up";
      pthState = PTH_TO_D;
      fsmstate = "pth_stand_up";
      break;

    case PTH_TO_D:
      if (!pthRotated) {
        if (usFrontM <= 100 && usRightM < 300 && yCurrent >= 13) {
          if ((lidarResult.frontWall && lidarResult.frontRightWall) || (lidarResult.frontRightWall && lidarResult.rightWall)) {
            pthRotated = true;
          } else {
            if (usRightM <= 70) pthCommand = "stop";
            else pthCommand = "step right";
          }
        } else if (usRightM <= 35) {
          pthCommand = "step left";
        } else if (usLeftM <= 35) {
          pthCommand = "step right";
        } else if (usFrontM <= 70) {
          if (currentRollDeg >= -20 || currentPitchDeg >= -20) {
            if (usFrontM <= 50) pthCommand = "step back";
            else pthCommand = "step left";
          } else pthCommand = "forward";
        } else {
          if (!isYawClose(currentYawDeg, startYawDeg + 4, 5)) {
            if (pth_YawErr < 0) {
              pthCommand = "rotate left";
            } else {
              pthCommand = "rotate right";
            }
          } else {
            pthCommand = "forward";
          }
        }
      } else {
        if (usRightM <= 50) {
          pthCommand = "step left";
        } else if (isYawClose(currentYawDeg, startYawDeg - 90)) {
          pthCommand = "stop";
          pthState = PTH_D_TO_CANAL;
          
          pthRotated = false;
        } else {
          if (pth_YawErr < 0) {
            pthCommand = "rotate left";
          } else {
            pthCommand = "rotate right";
          }
        }
      }
      fsmstate = "pth_to_d";
      break;

    case PTH_D_TO_CANAL:
      wallFollowingFSM(2, 50, 15);

      if (usFrontM <= 50 && usLeftM > 80 && !pth_obstacle_override) {
        pth_obstacle_override = true;
      }

      // Stay in override until obstacle cleared AND back near wall
      if (pth_obstacle_override) {
        if (usFrontM <= 50) {
          pthCommand = "step left";
        } else if (usRightM > 65) {
          pthCommand = "forward";  // Keep going until closer to wall
        } else {
          pth_obstacle_override = false;  // Now close enough to wall, resume normal
        }
        break;
      }

      if (!pthRotated) {
        if (usFrontM <= 50) pthCommand = "stop";
        else if (wfCommand == "no wall") pthRotated = true;
        else {
          if (!isYawClose(currentYawDeg, startYawDeg - 90, 10)) {
            if (pth_YawErr < 0) {
              pthCommand = "rotate left";
            } else {
              pthCommand = "rotate right";
            }
          } else {
            pthCommand = wfCommand;
          }
        }
      } else {
        if (isYawClose(currentYawDeg, startYawDeg)) {
          if (usRightM < 200) {
            pthCommand = "stop";
            pthState = PTH_THROUGH_CANAL;
            pthRotated = false;
          } else {
            pthCommand = "forward";
          }
        } else {
          if (pth_YawErr < 0) {
            pthCommand = "rotate left";
          } else {
            pthCommand = "rotate right";
          }
        }
      }
      fsmstate = "pth_d_to_canal";
      break;

    case PTH_THROUGH_CANAL:
      if (!pthRotated) {
        if (usFrontM <= 50) pthCommand = "step left";
        else if (usRightM >= 200) pthCommand = "forward";
        else pthRotated = true;
      } else if (!pthRotated2) {
        wallFollowingFSM(2, 60, 15);

        if (usFrontM <= 50 && usLeftM > 80 && !pth_obstacle_override) {
          pth_obstacle_override = true;
        }

        // Stay in override until obstacle cleared AND back near wall
        if (pth_obstacle_override) {
          if (usFrontM <= 50) {
            pthCommand = "step left";
          } else if (usRightM > 75) {
            pthCommand = "forward";  // Keep going until closer to wall
          } else {
            pth_obstacle_override = false;  // Now close enough to wall, resume normal
          }
          break;
        }

        if (wfCommand == "no wall" && usRightM >= 200) {
          pthRotated2 = true;
        } else if (usFrontM <= 100) {
          pthCommand = "stop";
        } else {
          if (!isYawClose(currentYawDeg, startYawDeg, 5)) {
            if (pth_YawErr < 0) {
              pthCommand = "rotate left";
            } else {
              pthCommand = "rotate right";
            }
          } else {
            pthCommand = wfCommand;
          }
        }
      } else {
        if (isYawClose(currentYawDeg, startYawDeg + 9)) {
          pthCommand = "stop";
          pthState = PTH_CANAL_TO_K;
          pthRotated = false;
          pthRotated2 = false;
        } else {
          if (pth_YawErr < 0) {
            pthCommand = "rotate left";
          } else {
            pthCommand = "rotate right";
          }
        }
      }
      fsmstate = "pth_through_canal";
      break;

    case PTH_CANAL_TO_K:
      if (!pthRotated) {
        if (usRightM <= 100) {
          if (currentRollDeg >= 10 || currentPitchDeg >= 10) pthCommand = "forward";
          else pthCommand = "step left";
        } else if (usFrontM <= 100) {
          if (usFrontM <= 50) {
            if ((lidarResult.frontRightWall && lidarResult.frontWall) || (lidarResult.frontWall && lidarResult.frontLeftWall)) pthRotated = true;
            else pthCommand = "step left";
          } else {
            pthCommand = "forward";
          }
        } else {
          if (!isYawClose(currentYawDeg, startYawDeg + 9, 5)) {
            if (pth_YawErr < 0) {
              pthCommand = "rotate left";
            } else {
              pthCommand = "rotate right";
            }
          } else {
            pthCommand = "forward";
          }
        }
        //}
      } else {
        if (isYawClose(currentYawDeg, startYawDeg + 90)) {
          pthCommand = "stop";
          pthState = PTH_K_TO_H;
          pthRotated = false;
        } else {
          if (pth_YawErr < 0) {
            pthCommand = "rotate left";
          } else {
            pthCommand = "rotate right";
          }
        }
      }
      fsmstate = "pth_canal_to_k";
      break;

    case PTH_K_TO_H:
      wallFollowingFSM(1, 100, 20);
      if (!pthRotated) {
        if (usFrontM <= 100) {
          if ((lidarResult.frontWall && lidarResult.frontRightWall) || (lidarResult.frontWall && lidarResult.frontLeftWall)) {
            pthRotated = true;
            pthRotationTarget = normalizeAngle(startYawDeg + 180);
          } else if (usFrontM <= 100 && usRightM > 80 && !pth_obstacle_override) {
            pth_obstacle_override = true;
          }
          if (pth_obstacle_override) {
            if (usFrontM <= 100) {
              pthCommand = "step right";
            } else if (usLeftM > 120) {
              pthCommand = "forward";  // Keep going until closer to wall
            } else {
              pth_obstacle_override = false;  // Now close enough to wall, resume normal
            }
            break;
          }
        } else {
          if (!isYawClose(currentYawDeg, startYawDeg + 90, 10)) {
            if (pth_YawErr < 0) {
              pthCommand = "rotate left";
            } else {
              pthCommand = "rotate right";
            }
          } else {
            if (wfCommand == "no wall") pthCommand = "forward";
            else pthCommand = wfCommand;
          }
        }
      } else {
        if (isYawClose(currentYawDeg, pthRotationTarget)) {
          pthCommand = "stop";
          pthState = PTH_H_TO_POLE;
          pthRotated = false;
        } else {
          if (pth_YawErr < 0) {
            pthCommand = "rotate left";
          } else {
            pthCommand = "rotate right";
          }
        }
      }
      fsmstate = "pth_k_to_h";
      break;

    case PTH_H_TO_POLE:
      wallFollowingFSM(1, 250, 15);
      if (usFrontM <= 50) pthCommand = "step left";
      else if (usRightM <= 60) {
        pthCommand = "stop";
        pthState = PTH_LAY_DOWN;
      } else {
        if (!isYawClose(currentYawDeg, pthRotationTarget, 10)) {
          if (pth_YawErr < 0) {
            pthCommand = "rotate left";
          } else {
            pthCommand = "rotate right";
          }
        } else {
          if (wfCommand == "no wall") pthCommand = "step left";
          else pthCommand = wfCommand;
        }
      }
      fsmstate = "pth_h_to_pole";
      break;

    case PTH_LAY_DOWN:
      lockLaydownStand();
      delay(3000);
      lockLaydownStand();
      ptharrived = true;
      pthCommand = "stop";
      pthState = PTH_IDLE;
      fsmstate = "arrived";
      break;
  }
}

//--------------------------

// -----------------------------------------------------------------------------
// Setup / Loop (minimaler Integrationsrahmen)
// Du kannst den Demo-Teil im loop später komplett ersetzen.
void setup() {
  //Initialisation
  Serial.begin(115200);

  disp.begin();

  // Controller I2C Devices
  if (!initDAC()) Serial.println("initDAC failed");
  if (!initExpMod()) Serial.println("initExpMod failed");

  // WebHMI
  hmi.begin();
  hmi.setTitle("Robo Dog");
  hmi.enableCycleStats(true);

  disp.show("Out", "Ready");
  hmi.showValue("Output", "Ready");
  //hmi.showButton("Start", false); // omited 2026_01_17_
  hmi.showButton("wake up dance", false);
  hmi.showButton("stop nav", false);
  hmi.showButton("calibrate imu", false);
  hmi.enableOperatorUI(true);                                     //2026_01_17_added
  hmi.opAddPrimaryButton("OP.SYSTEM.START", "Start: Post to H");  //2026_01_17_added
  hmi.opShowText("UI.StateText", "System is booting.");           //2026_01_17_added
  imu.IMUBegin();
  us.begin();
  us.setMaxDistanceCm(300);
  us.setTimeoutUs(17500);
  gps.begin();


  //Startup Routine

  // IMU
  delay(300);
  imu.SetCampusYawOffsetDeg(0.0f);
  imu.UpdateAllData();
  startYawDeg = imu.ReadYawCampus();

  // GPS
  delay(600);
  gps.setTouchUpOriginThdDefault(false);

  // Lidar
  lidarAlign_init(rxPin, txPin);
  hmi.opShowText("UI.StateText", "Setup done.");  //2026_01_17_added
}

void loop() {
  // neccessary refreshers etc.
  hmi.tick();
  out.beginCycle();  // alow only one command in the loop
  us.update();
  gps.poll();
  // get sensor values
  // GPS
  gps.getCampusXY(xCurrent, yCurrent);  // get current gps values in xCurrent and yCurrent
  // IMU
  imu.UpdateAllData();
  currentYawDeg = imu.ReadYawCampus();
  currentPitchDeg = imu.ReadPitch();
  currentRollDeg = imu.ReadRoll();

  // Ultrasonic
  usFrontM = us.readFrontCm();
  //usAngeled = us.readFrontAngledCm();  // alternativ je nach Klasse: getDropOffMeters() / getFrontAngledMeters()
  //usRearM = us.readBackCm();
  usRightM = us.readRightCm();
  usLeftM = us.readLeftCm();
  // LiDAR
  lidarAlign_update();
  lidarResult = lidarAlign_getDetect();
  //

  if (hmi.getButtonState("wake up dance")) {
    initialcheckuproutine();
    hmi.setButtonState("wake up dance", false);
  }
  /* 2026_01_17_replaced
    if (hmi.getButtonState("Start")) {
      if (!pth_navStart) {
        pthState = PTH_IDLE;
        pth_navStart = true;
      }
      hmi.setButtonState("Start", false);
    }
  */
  // Start replacement
  char opCmd[32];

  if (hmi.opPopCommand(opCmd, sizeof(opCmd))) {
    if (strcmp(opCmd, "OP.SYSTEM.START") == 0) {

      if (!pth_navStart) {
        pthState = PTH_IDLE;
        pth_navStart = true;
        pthRotated = false;
        pthRotated2 = false;
      }
    }
  }
  //End Replacement

  if (millis() - lastFsmUpdate >= 50) {
    if (pth_navStart) {
      POST_TO_H_NAV_FSM();
      commandSet(pthCommand);
      lastFsmUpdate = millis();
      if (ptharrived) pth_navStart = false;
    }
  }

  if (hmi.getButtonState("stop nav")) {
    pthState = PTH_IDLE;
    pth_navStart = false;
    pthRotated = false;
    pthRotated2 = false;
    pthCommand = "stop";
    commandSet(pthCommand);
    hmi.setButtonState("stop nav", false);
  }

  if (hmi.getButtonState("calibrate imu")) {
    imu.CalibrateYaw(true);
    startYawDeg = imu.ReadYawCampus();
    hmi.setButtonState("calibrate imu", false);
    imu.CalibrateYaw(false);
  }


  if (millis() - lastHmiUpdate >= 250) {  // Only update HMI every 250ms
    hmi.showValue("State", fsmstate);

    uiStateText = (fsmstate.length() == 0 || fsmstate == "nil") ? String(kWaitForCommand) : fsmstate;  //2026_01_17_added
    hmi.opShowText("UI.StateText", uiStateText);                                                       //2026_01_17_added

    hmi.showValue("current_yaw", currentYawDeg);
    hmi.showValue("start_yaw", startYawDeg);
    hmi.showValue("frontUS", usFrontM);
    hmi.showValue("rightUS", usRightM);
    hmi.showValue("leftUS", usLeftM);
    hmi.showValue("gps_y", yCurrent);
    lastHmiUpdate = millis();
  }
}