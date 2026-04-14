#pragma once
#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// Motion commands returned to the navigation FSM
typedef enum {
  LIDAR_CMD_NONE = 0,
  LIDAR_CMD_STOP,
  LIDAR_CMD_ROTATE_LEFT,
  LIDAR_CMD_ROTATE_RIGHT,
  LIDAR_CMD_FORWARD
} LidarCmdType;

typedef struct {
  LidarCmdType type;
  float rateDeg_s;   // valid for rotate commands
  float speed_mps;   // valid for forward command
} LidarMotionCmd;

// ===== Free-space scan ("no wall/obstacle within range") =====
// A direction is considered "free" if all samples in a small window around that
// angle are either:
//  - >= minRange_m, or
//  - 0 (no return, treated as "beyond range")
//
// NOTE: If you want a more conservative behavior, treat 0 as "unknown" instead.

#ifndef LIDAR_MAX_FREE_RANGES
#define LIDAR_MAX_FREE_RANGES 12
#endif

typedef struct {
  int startDeg;      // inclusive [0..359]
  int endDeg;        // inclusive [0..359]
  int widthDeg;
  int centerDeg;
  bool uncertain;    // true if the window had only 0-returns (no hits)
} LidarFreeRange;

typedef struct {
  uint8_t count;
  LidarFreeRange ranges[LIDAR_MAX_FREE_RANGES];
} LidarFreeScanResult;

// What the navigation FSM wants to read
typedef struct {
  // Wall presence (8 sectors)
  bool frontWall;
  bool frontLeftWall;
  bool leftWall;
  bool backLeftWall;
  bool backWall;
  bool backRightWall;
  bool rightWall;
  bool frontRightWall;

  // Obstacle presence (8 sectors)
  bool frontObs;
  bool frontLeftObs;
  bool leftObs;
  bool backLeftObs;
  bool backObs;
  bool backRightObs;
  bool rightObs;
  bool frontRightObs;

  // Min distances (meters) for 8 sectors
  float frontMin_m;
  float frontLeftMin_m;
  float leftMin_m;
  float backLeftMin_m;
  float backMin_m;
  float backRightMin_m;
  float rightMin_m;
  float frontRightMin_m;

  // Wall line angles (deg) - still 4 cardinal directions
  float frontWallLineAngleDeg;
  float leftWallLineAngleDeg;
  float rightWallLineAngleDeg;
  float backWallLineAngleDeg;

  // Facing error to ideal (deg) - still 4 cardinal directions
  float frontWallErrDeg;
  float leftWallErrDeg;
  float rightWallErrDeg;
  float backWallErrDeg;

  // Health info
  uint32_t packets;
  uint32_t crcErrors;
  float crcRate;

  // Alignment gate (front wall latched)
  bool alignAllowed;
} LidarDetectResult;
// 0=Idle, 1=Coarse, 2=Fine
uint8_t lidarAlign_state();
float   lidarAlign_errorDeg();
bool    lidarAlign_canAlign();
bool    lidarAlign_takeDone();

// API
void lidarAlign_init(int rxPin, int txPin);   // uses Serial1 internally
void lidarAlign_update();

//pole detection
int lidarAlign_detectNarrowPole();

LidarDetectResult lidarAlign_getDetect();

// Returns current free-angle ranges for >= 12m (computed from latest scan).
LidarFreeScanResult lidarAlign_getFree12m();

void lidarAlign_startAlignLeft();
void lidarAlign_startAlignRight();
void lidarAlign_cancel();

// Rotate LEFT until the FRONT direction is free for >= 12m, then drive forward
// until an obstacle/wall appears within a safety distance.
void lidarAlign_startFreeDrive12m();
bool lidarAlign_freeDriveActive();
bool lidarAlign_freeDriveTakeDone();

LidarMotionCmd lidarAlign_getMotionCmd();

#ifdef __cplusplus
}
#endif
