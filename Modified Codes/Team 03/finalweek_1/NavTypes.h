#pragma once

enum Cmd : uint8_t { CMD_STOP, CMD_FWD, CMD_LEFT, CMD_RIGHT, CMD_ROT_L, CMD_ROT_R };

static inline const char* cmdText(Cmd c){
  switch (c) {
    case CMD_STOP: return "STOP";
    case CMD_FWD:  return "FWD";
    case CMD_LEFT: return "LEFT";
    case CMD_RIGHT:return "RIGHT";
    case CMD_ROT_L:return "ROT_L";
    case CMD_ROT_R:return "ROT_R";
    default:       return "STOP";
  }
}

struct IMU_US_Data {
  float heading=0, roll=0, pitch=0;
  int front=0, back=0, left=0, right=0, angle45=0;
  uint8_t sys_cal=0;
};

struct GPS_Data {
  double lat=0, lon=0, alt=0, x=0, y=0;
  int sats=0;
};

struct Pose2D {
  float x=0, y=0, heading=0;
};

struct SensorsSnap {
  IMU_US_Data imuUs = {};
  float frontWallDist_m = 999.0f;
};

struct NavState {
  int wpIndex=0;
  int phase=0;
  bool aligned=false;
  int alignCount=0;
};
