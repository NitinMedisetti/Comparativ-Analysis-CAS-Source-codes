#include "app_state.h"

#include <math.h>

#include "timebase.h"

namespace {
inline void init_bins(AppState& state) {
  for (float& bin : state.lidarBins) {
    bin = NAN;
  }
  state.lidarBinsStamp = {};
}

inline void init_lidar_ui(AppState& state) {
  state.lidarUiSegmentCount = 0;
  for (auto& seg : state.lidarUiSegments) {
    seg = {};
  }
}

inline void reset_navigation(NavigationState& nav) {
  nav = {};
  nav.dest_lat = NAN;
  nav.dest_lng = NAN;
  nav.ref_lat = NAN;
  nav.ref_lng = NAN;
  nav.ref_valid = false;
  nav.ref_active = false;
  nav.route_seq = 0;
  nav.aim_yaw_deg = NAN;
  nav.distance_to_dest_m = NAN;
  nav.bearing_to_dest_deg = NAN;
  nav.segment_distance_m = NAN;
  nav.segment_remaining_m = NAN;
  nav.segment_eta_s = NAN;
  nav.segment_speed_mps = NAN;
  nav.segment_index = 0;
  nav.segment_total = 0;
  nav.status = "idle";
  nav.decision = "idle";
  nav.motion_percent = 0;
  nav.target_label = "";
  nav.target_key = "";
  nav.phase = "";
  nav.obstacles.possible_obstacles = "";
  nav.current_phase_enum = NavPhase::BOOT_CHECK;
  nav.path_len = 0;
  nav.current_path_idx = 0;
  nav.status_line_1 = "";
  nav.status_line_2 = "";
}
}  // namespace

AppStateBuffer::AppStateBuffer() {
  read_ = &buffers_[0];
  write_ = &buffers_[1];
  app_state_reset(*read_);
  app_state_reset(*write_);
}

AppState& AppStateBuffer::writable() { return *write_; }
AppState& AppStateBuffer::readable() { return *read_; }
const AppState& AppStateBuffer::readable() const { return *read_; }

void AppStateBuffer::reset_bins(AppState& state) {
  init_bins(state);
  init_lidar_ui(state);
}

void AppStateBuffer::publish(uint8_t status) {
  (void)status;
  // Swap readable/writable buffers. Callers typically hold `g_state_mutex` around
  // writes; readers can then consume the most recent published snapshot.
  AppState* old_read = read_;
  read_ = write_;
  write_ = old_read;
  // Clear large/derived fields in the next writable buffer so the next frame starts
  // from a known state (keeps LiDAR bins/UI segments from "sticking").
  reset_bins(*write_);
}

void app_state_reset(AppState& state) {
  state.gps = {};
  state.imu = {};
  state.us = {};
  state.sys = {};
  init_bins(state);
  init_lidar_ui(state);
  state.lidar_sweep_id = 0;
  state.lidar_packet_seq = 0;
  state.lidarHealth = {};
  state.units = "mm";
  state.stabilize = false;
  state.cam_url = "";
  state.cam_status = "CAM OFFLINE";
  state.oled_label = "Group 2";
  reset_navigation(state.nav);
}
