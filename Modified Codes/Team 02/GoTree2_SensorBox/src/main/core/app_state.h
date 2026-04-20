#pragma once

#include <Arduino.h>

// Shared state snapshot used across tasks (sensors, navigation, UI/OLED).
//
// Conventions:
// - NaN generally means "unknown / not valid yet".
// - `Stamp` is a lightweight "updated at" marker: monotonic time + sequence counter.

struct Stamp {
  // Monotonic time in microseconds (see `now_us()` in `timebase.h`).
  uint64_t t_us = 0;
  // Incremented whenever this block is refreshed.
  uint32_t seq = 0;
  uint8_t status = 0;
};

struct GPSBlock {
  // Best-effort GPS solution from TinyGPS++.
  Stamp stamp;
  bool valid = false;
  double lat = NAN;
  double lng = NAN;
  double alt_m = NAN;
  double speed_kmh = NAN;
  double heading_deg = NAN;
  double hdop = NAN;
  float temp_c = NAN;
  uint32_t sats = 0;
  uint32_t age_ms = 0;
  String utc = "null";
  String date = "null";
};

struct IMUBlock {
  // BNO055 Euler angles + raw vectors; values are NaN until the IMU is online.
  Stamp stamp;
  float yaw = NAN;
  float pitch = NAN;
  float roll = NAN;
  float tempC = NAN;
  float linear_accel_x = NAN;
  float linear_accel_y = NAN;
  float linear_accel_z = NAN;
  float accel_x = NAN;
  float accel_y = NAN;
  float accel_z = NAN;
  float gyro_x = NAN;
  float gyro_y = NAN;
  float gyro_z = NAN;
  float mag_x = NAN;
  float mag_y = NAN;
  float mag_z = NAN;
  uint8_t cal_sys = 0;
  uint8_t cal_gyro = 0;
  uint8_t cal_accel = 0;
  uint8_t cal_mag = 0;
  bool calibrated = false;
  bool valid = false;
};

struct USBlock {
  // Ultrasonic ranges (millimeters). A value of 0 typically means "no echo / out of range".
  Stamp stamp;
  long front_mm = 0;
  long back_mm = 0;
  long left_mm = 0;
  long right_mm = 0;
  long d45_mm = 0;
};

struct SystemBlock {
  // Basic system health/telemetry for the web UI.
  Stamp stamp;
  uint32_t uptime_ms = 0;
  int wifi_rssi = 0;
  size_t free_heap = 0;
  size_t largest_block = 0;
};

struct LiDARHealth {
  // LiDAR stream health computed from packet timing.
  Stamp stamp;
  int rpm = 0;
  uint32_t last_ms = 0;
  bool frame_ok = false;
  float frame_hz = 0.0f;
};

enum LidarUiSegmentType : uint8_t {
  kLidarUiSegmentDot = 0,
  kLidarUiSegmentLine = 1
};

struct LidarUiSegment {
  float start_deg = NAN;
  float end_deg = NAN;
  float mean_deg = NAN;
  uint16_t start_mm = 0;
  uint16_t end_mm = 0;
  uint16_t mean_mm = 0;
  uint8_t start_int = 0;
  uint8_t end_int = 0;
  uint8_t mean_int = 0;
  uint8_t type = kLidarUiSegmentDot;
};

constexpr uint8_t kMaxLidarUiSegments = 64;

struct ObstacleSnapshot {
  // Conservative "blocked" flags derived from ultrasonic + LiDAR sectors.
  bool front_blocked = false;
  bool left_blocked = false;
  bool right_blocked = false;
  bool rear_blocked = false;
  bool hard_stop = false;
  String possible_obstacles;
  float closest_dist_mm = NAN;
};

enum class NavPhase : uint8_t {
  BOOT_CHECK = 0,
  IDLE = 1,

  GOTO_H_INIT = 2,
  GOTO_H_LEG1_DRIVE = 3,
  GOTO_H_TURN1 = 4,
  GOTO_H_ALIGN1 = 6,
  GOTO_H_LEG2_DRIVE = 7,
  GOTO_H_TURN2 = 8,
  GOTO_H_FIND_WALL2 = 9,
  GOTO_H_ALIGN2 = 10,
  GOTO_H_LEG3_DRIVE = 11,
  GOTO_H_TURN3 = 12,
  GOTO_H_ALIGN3 = 13,
  GOTO_H_LEG4_DRIVE = 14,
  GOTO_H_TURN4 = 15,
  GOTO_H_ALIGN4 = 16,
  GOTO_H_FINAL_APP = 17,
  GOTO_H_DONE = 18,

  // New phases (appended).
  GOTO_H_WAIT_WALL2 = 19,
  GOTO_H_LEG3_OPEN_DRIVE = 20,
};

struct NavigationState {
  // Navigation status produced by `nav_tick()` (see `core/decision_alg.*`).
  bool active = false;
  bool system_started = false;
  bool heading_ready = false;
  bool gps_ready = false;
  bool ready_for_goto = false;
  String target_key;
  String target_label;
  String phase;
  String status;
  String decision;
  uint8_t motion_percent = 0;  // 0 = use default motor speed
  float aim_yaw_deg = NAN;
  double dest_lat = NAN;
  double dest_lng = NAN;
  double ref_lat = NAN;
  double ref_lng = NAN;
  bool ref_valid = false;
  bool ref_active = false;
  // Incremented whenever a mission/route is restarted so the UI can detect changes.
  uint8_t route_seq = 0;
  float distance_to_dest_m = NAN;
  float bearing_to_dest_deg = NAN;
  float segment_distance_m = NAN;
  float segment_remaining_m = NAN;
  float segment_eta_s = NAN;
  float segment_speed_mps = NAN;
  uint8_t segment_index = 0;
  uint8_t segment_total = 0;
  uint32_t updated_ms = 0;
  ObstacleSnapshot obstacles;

  NavPhase current_phase_enum = NavPhase::IDLE;
  // Compact path representation: waypoint ids + current index (used by the UI).
  uint8_t path_ids[32] = {};
  uint8_t path_len = 0;
  uint8_t current_path_idx = 0;
  String status_line_1;
  String status_line_2;
};

struct AppState {
  // Sensor blocks.
  GPSBlock gps;
  IMUBlock imu;
  USBlock us;
  SystemBlock sys;

  // 360-degree LiDAR map (millimeters), plus optional UI segments.
  float lidarBins[360];
  Stamp lidarBinsStamp;
  uint32_t lidar_sweep_id = 0;
  uint32_t lidar_packet_seq = 0;
  LiDARHealth lidarHealth;
  LidarUiSegment lidarUiSegments[kMaxLidarUiSegments];
  uint8_t lidarUiSegmentCount = 0;

  String units;
  bool stabilize = false;
  String cam_url;
  String cam_status;
  String oled_label;

  // Navigation output (updated by the decision loop).
  NavigationState nav;
};

class AppStateBuffer {
 public:
  // Double-buffered AppState to reduce lock contention between tasks.
  AppStateBuffer();

  AppState& writable();
  AppState& readable();
  const AppState& readable() const;

  void publish(uint8_t status = 0);

 private:
  void reset_bins(AppState& state);

  AppState buffers_[2];
  AppState* read_ = nullptr;
  AppState* write_ = nullptr;
};

void app_state_reset(AppState& state);
