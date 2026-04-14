#ifndef TEAM4TYPES_H
#define TEAM4TYPES_H

#include <Arduino.h>

// ================================================================
// AutoState Enum - All possible robot states
// ================================================================
enum AutoState {
  IDLE,
  TURNING,
  
  // Zone 1+2 states
  Z12_FORWARD_TO_CORNER,
  Z12_ALIGN_RIGHT_70,
  Z12_FOLLOW_RIGHT_70,
  Z12_FORWARD_TO_RIGHT_WALL_2,
  Z12_ALIGN_RIGHT_50,
  Z12_FOLLOW_RIGHT_50,
  
  // Zone 3 states
  Z3_FORWARD_TO_FRONT,
  Z3_ALIGN_LEFT_100,
  Z3_FOLLOW_LEFT_WALL,
  Z3_FORWARD_TO_FRONT_2,
  Z3_ALIGN_LEFT_200,
  Z3_DONE,
  
  // Obstacle avoidance states
  OBSTACLE_STOP,
  OBSTACLE_STEP_BACK,
  BENCH_STEP_RIGHT
};

// ================================================================
// Telemetry Struct - Sensor data and state information
// ================================================================
struct Telemetry {
  // Ultrasonic sensors (cm)
  float f;  // front
  float b;  // back
  float l;  // left
  float r;  // right
  
  // IMU
  float yaw;       // relative yaw (degrees)
  float yawAbs;    // absolute yaw (degrees)
  
  // State
  char state[32];   // current state name
  char action[24];  // current action
  bool autoEnabled; // auto mode enabled
  
  // Counters (for state machine confirmations)
  int c1;
  int c2;
  int c3;
  int c4;
  
  // Turn info
  float turnRemaining;  // degrees remaining in turn
  
  // Waypoints
  uint8_t wpPassed;  // number of waypoints passed (0-2)
  
  // Constructor - initialize all values
  Telemetry() : 
    f(251), b(251), l(251), r(251),
    yaw(0), yawAbs(0),
    autoEnabled(false),
    c1(0), c2(0), c3(0), c4(0),
    turnRemaining(0),
    wpPassed(0)
  {
    strcpy(state, "IDLE");
    strcpy(action, "STOP");
  }
};

#endif // TEAM4TYPES_H