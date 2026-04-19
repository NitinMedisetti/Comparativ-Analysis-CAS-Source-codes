#include "decision_alg.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "core/controller_state.h"
#include "sensors/sensor_us.h"

// Navigation + decision logic.
//
// Responsibilities:
// - Maintain a simple mission state machine (see `NavPhase` in `app_state.h`).
// - Choose a motion command (STOP/drive/rotate) based on target + sensor health.
// - Apply obstacle avoidance overrides (ultrasonic + LiDAR sectors).
// - Publish a compact `NavigationState` snapshot for the UI and OLED.

namespace {

struct LatLng {
  double lat;
  double lng;
};

struct XY {
  double x;
  double y;
};

struct BuildingPoly {
  const char* name;
  const LatLng* pts;
  uint8_t count;
};

// Building footprints from `GoTree2_SensorBox/data/detailed_gps_data.json` (zones.points),
// used to reject waypoint-to-waypoint edges that would cross a building.
static constexpr LatLng kPolyA[] = {{48.83055477, 12.95398045},
                                    {48.83036155, 12.95452506},
                                    {48.83023838, 12.95442322},
                                    {48.83043278, 12.95387903}};
static constexpr LatLng kPolyB[] = {{48.83032662, 12.95461385},
                                    {48.83012539, 12.95518119},
                                    {48.83002186, 12.95509788},
                                    {48.83022439, 12.95452802}};
static constexpr LatLng kPolyC[] = {{48.83009249, 12.95527556},
                                    {48.82964142, 12.95654342},
                                    {48.82951902, 12.95644383},
                                    {48.8299891, 12.95518925}};
static constexpr LatLng kPolyD1[] = {{48.8301491, 12.95363846},
                                     {48.82995177, 12.95419147},
                                     {48.82982308, 12.95408531},
                                     {48.83002521, 12.95353614}};
static constexpr LatLng kPolyD2[] = {{48.82994831, 12.95420265},
                                     {48.82971801, 12.95485065},
                                     {48.82959424, 12.95475068},
                                     {48.82982021, 12.95409655}};
static constexpr LatLng kPolyE1[] = {{48.82968642, 12.95493965},
                                     {48.82947304, 12.95553861},
                                     {48.82934615, 12.95544258},
                                     {48.82956428, 12.95483691}};
static constexpr LatLng kPolyE2[] = {{48.82946899, 12.95555075},
                                     {48.82923371, 12.95620865},
                                     {48.82911034, 12.95610622},
                                     {48.82934144, 12.955455}};
static constexpr LatLng kPolyF[] = {{48.83032497, 12.95402338},
                                    {48.83020069, 12.95436911},
                                    {48.83001067, 12.95421198},
                                    {48.83013728, 12.95386536}};
static constexpr LatLng kPolyG[] = {{48.82968473, 12.95579184},
                                    {48.82957456, 12.95609908},
                                    {48.82939222, 12.95594987},
                                    {48.8295027, 12.95564286}};
static constexpr LatLng kPolyH[] = {{48.82966778, 12.95406313},
                                    {48.82956667, 12.95434354},
                                    {48.82936484, 12.95417469},
                                    {48.8294659, 12.95389458}};
static constexpr LatLng kPolyI[] = {{48.82941993, 12.95474496},
                                    {48.82922209, 12.95529116},
                                    {48.82904442, 12.95514394},
                                    {48.82924214, 12.95459641}};
static constexpr LatLng kPolyJ[] = {{48.82970063, 12.95332201},
                                    {48.82951673, 12.95383731},
                                    {48.8293067, 12.95366474},
                                    {48.82948981, 12.95314788}};
static constexpr LatLng kPolyK[] = {{48.8293688, 12.95424493},
                                    {48.82927134, 12.95451562},
                                    {48.82906077, 12.95433907},
                                    {48.82915903, 12.95406994}};
static constexpr LatLng kPolyL[] = {{48.82899643, 12.9552736},
                                    {48.82879382, 12.95584388},
                                    {48.82856762, 12.95565854},
                                    {48.82877088, 12.95508882}};
static constexpr LatLng kPolyStore[] = {{48.829853817143885, 12.955214068666692},
                                        {48.829769250828605, 12.955145865852188},
                                        {48.8296788, 12.95526913},
                                        {48.82967517, 12.95542326},
                                        {48.82975966, 12.95549127},
                                        {48.82981016, 12.95541377}};
static constexpr LatLng kPolyPool1[] = {{48.83000454575097, 12.954304152449026},
                                        {48.82989939365709, 12.954619626603147},
                                        {48.83008538127336, 12.954754401953522},
                                        {48.8301960579044, 12.954456627760862}};
static constexpr LatLng kPolyPool2[] = {{48.82958954399032, 12.955414387537303},
                                        {48.82955036326547, 12.955526930557738},
                                        {48.829724182975156, 12.955667609332352},
                                        {48.829764075938385, 12.955557230601357}};

static constexpr BuildingPoly kBuildings[] = {
    {"A", kPolyA, (uint8_t)(sizeof(kPolyA) / sizeof(kPolyA[0]))},
    {"B", kPolyB, (uint8_t)(sizeof(kPolyB) / sizeof(kPolyB[0]))},
    {"C", kPolyC, (uint8_t)(sizeof(kPolyC) / sizeof(kPolyC[0]))},
    {"D1", kPolyD1, (uint8_t)(sizeof(kPolyD1) / sizeof(kPolyD1[0]))},
    {"D2", kPolyD2, (uint8_t)(sizeof(kPolyD2) / sizeof(kPolyD2[0]))},
    {"E1", kPolyE1, (uint8_t)(sizeof(kPolyE1) / sizeof(kPolyE1[0]))},
    {"E2", kPolyE2, (uint8_t)(sizeof(kPolyE2) / sizeof(kPolyE2[0]))},
    {"F", kPolyF, (uint8_t)(sizeof(kPolyF) / sizeof(kPolyF[0]))},
    {"G", kPolyG, (uint8_t)(sizeof(kPolyG) / sizeof(kPolyG[0]))},
    {"H", kPolyH, (uint8_t)(sizeof(kPolyH) / sizeof(kPolyH[0]))},
    {"I", kPolyI, (uint8_t)(sizeof(kPolyI) / sizeof(kPolyI[0]))},
    {"J", kPolyJ, (uint8_t)(sizeof(kPolyJ) / sizeof(kPolyJ[0]))},
    {"K", kPolyK, (uint8_t)(sizeof(kPolyK) / sizeof(kPolyK[0]))},
    {"L", kPolyL, (uint8_t)(sizeof(kPolyL) / sizeof(kPolyL[0]))},
    {"Store", kPolyStore, (uint8_t)(sizeof(kPolyStore) / sizeof(kPolyStore[0]))},
    {"Pool1", kPolyPool1, (uint8_t)(sizeof(kPolyPool1) / sizeof(kPolyPool1[0]))},
    {"Pool2", kPolyPool2, (uint8_t)(sizeof(kPolyPool2) / sizeof(kPolyPool2[0]))},
};

}  // namespace

namespace nav {

bool g_system_started = false;

namespace {
NavigationState g_nav;

enum class NavCommand : uint8_t {
  None = 0,
  StartSystem,
  Stop,
  Hold,
  GotoPost,
  GotoH,
};

static portMUX_TYPE s_nav_cmd_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile NavCommand s_pending_cmd = NavCommand::None;

static inline NavCommand parse_nav_command(const char* action) {
  if (!action) return NavCommand::None;
  if (strcmp(action, "start_system") == 0) return NavCommand::StartSystem;
  if (strcmp(action, "stop") == 0) return NavCommand::Stop;
  if (strcmp(action, "hold") == 0) return NavCommand::Hold;
  if (strcmp(action, "goto_post") == 0) return NavCommand::GotoPost;
  if (strcmp(action, "goto_h") == 0) return NavCommand::GotoH;
  return NavCommand::None;
}

static inline NavCommand take_pending_command() {
  NavCommand cmd = NavCommand::None;
  portENTER_CRITICAL(&s_nav_cmd_mux);
  cmd = s_pending_cmd;
  s_pending_cmd = NavCommand::None;
  portEXIT_CRITICAL(&s_nav_cmd_mux);
  return cmd;
}

struct Kalman1D {
  bool init = false;
  double x = NAN;
  double p = 1.0;
  double q = 1e-10;  // process noise (deg^2)
  double r = 2.5e-9; // measurement noise (deg^2)

  void reset() {
    init = false;
    x = NAN;
    p = 1.0;
  }

  double update(double z) {
    if (!isfinite(z)) return x;
    if (!init) {
      init = true;
      x = z;
      p = 1.0;
      return x;
    }
    p += q;
    const double k = p / (p + r);
    x = x + k * (z - x);
    p = (1.0 - k) * p;
    return x;
  }
};

struct Kalman2D {
  Kalman1D lat;
  Kalman1D lng;

  void reset() {
    lat.reset();
    lng.reset();
  }

  bool valid() const { return lat.init && lng.init && isfinite(lat.x) && isfinite(lng.x); }

  void update(double raw_lat, double raw_lng, double* out_lat, double* out_lng) {
    const double f_lat = lat.update(raw_lat);
    const double f_lng = lng.update(raw_lng);
    if (out_lat) *out_lat = f_lat;
    if (out_lng) *out_lng = f_lng;
  }
};

struct MotionCmd {
  const char* decision;
  uint8_t percent;

  constexpr MotionCmd(const char* d = "STOP", uint8_t p = 0) : decision(d), percent(p) {}
};

enum class ObstacleType { None, Unknown, Wall };
enum class ObsState {
  CHECK,
  ASSESS_TYPE,       // Deciding Static vs Dynamic
  WAIT_DYNAMIC,      // Waiting for path to clear
  AVOID_STEP_OUT,    // Stepping away (tracking time)
  AVOID_FORWARD,     // Moving past the object
  AVOID_STEP_IN,     // Returning to route (using tracked time)
  CLEAR
};

constexpr uint32_t kInitSlowMs = 4000;
constexpr uint32_t kPhaseTransitionPauseMs = 1000;
constexpr uint32_t kLeg3RampMs = 2000;
constexpr uint32_t kTurnTimeoutMs = 8000;
constexpr float kYawTolDeg = 2.0f;
constexpr float kDriveYawTolDeg = 3.0f;

constexpr long kWallTargetMm = 1200;
constexpr long kWallTolMm = 300;
constexpr long kWallWideTargetMm = 2500;
constexpr long kWallWideTolMm = 200;
constexpr long kLeg3WallTargetMm = 1500;
constexpr long kDriveRealignTriggerMm = 500;

constexpr long kNoWallMm = 4500;
constexpr uint32_t kWallFollowStepMs = 220;
constexpr long kLeg1LeftSeenMm = 1500;
constexpr long kLeg1FrontConfirmTolMm = 200;

constexpr float kLidarClearanceInitMm = 4000.0f;
constexpr float kLidarObstacleMm = 3000.0f;
constexpr float kLidarHardStopMm = 500.0f;
constexpr uint8_t kLidarObstacleMinBins = 3;

constexpr long kUsSafetyStopMm = 300;

constexpr float kObstacleMinDistMm = 20.0f;
constexpr float kObstacleMaxDistMm = 1000.0f;
constexpr float kObstacleClusterGapMm = 100.0f;
constexpr uint8_t kObstacleClusterMinPoints = 3;
constexpr float kObstacleWallSpanDeg = 20.0f;
constexpr float kObstacleLineRatio = 0.12f;
constexpr float kObstacleDynamicDeltaMm = 200.0f;
constexpr float kObstacleClearHysteresisMm = 150.0f;
constexpr uint32_t kObstacleClearConfirmMs = 500;
constexpr uint32_t kObstacleAssessMs = 1000;
constexpr float kObstacleClearMm = kObstacleMaxDistMm + kObstacleClearHysteresisMm;

constexpr double kEarthRadiusM = 6371000.0;
constexpr double kDegToRad = PI / 180.0;
constexpr double kRadToDeg = 180.0 / PI;

constexpr double kGotoHStartLat = 48.830041588956874;
constexpr double kGotoHStartLng = 12.954990534110266;
constexpr double kGotoHTurn1Lat = 48.82974574830391;
constexpr double kGotoHTurn1Lng = 12.954764741754161;
constexpr double kGotoHTurn2Lat = 48.82971160282983;
constexpr double kGotoHTurn2Lng = 12.954866071758971;
constexpr double kGotoHTurn3Lat = 48.829285074668974;
constexpr double kGotoHTurn3Lng = 12.954506998467338;
constexpr double kGotoHTurn4Lat = 48.82938576319074;
constexpr double kGotoHTurn4Lng = 12.954222594468177;
constexpr double kGotoHFinalAppLat = 48.8294576221368;
constexpr double kGotoHFinalAppLng = 12.954285158729192;

constexpr double kLeg1TargetLat = kGotoHTurn1Lat;
constexpr double kLeg1TargetLng = kGotoHTurn1Lng;
constexpr float kLeg1GpsProximityM = 6.0f;
constexpr double kLeg2TargetLat = kGotoHTurn2Lat;
constexpr double kLeg2TargetLng = kGotoHTurn2Lng;
constexpr double kLeg3TargetLat = kGotoHTurn3Lat;
constexpr double kLeg3TargetLng = kGotoHTurn3Lng;
constexpr float kLeg3GpsProximityM = 8.0f;
constexpr double kLeg4TargetLat = kGotoHTurn4Lat;
constexpr double kLeg4TargetLng = kGotoHTurn4Lng;
constexpr double kFinalAppTargetLat = kGotoHFinalAppLat;
constexpr double kFinalAppTargetLng = kGotoHFinalAppLng;

constexpr double kGpsTrustMaxHdop = 2.0;
constexpr uint32_t kGpsTrustMinSats = 6;

static inline float wrap_deg(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

static inline int wrap_deg_i(int deg) {
  int d = deg % 360;
  if (d < 0) d += 360;
  return d;
}

static inline float angle_delta_deg(float from_deg, float to_deg) {
  float d = wrap_deg(to_deg) - wrap_deg(from_deg);
  if (d > 180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

static inline bool aligned_yaw(float yaw_deg, float target_deg, float tol_deg) {
  if (!isfinite(yaw_deg) || !isfinite(target_deg)) return false;
  return fabsf(angle_delta_deg(yaw_deg, target_deg)) <= tol_deg;
}

static inline bool drive_yaw_align_cmd(float yaw_deg, float target_deg, float tol_deg, MotionCmd* out_cmd) {
  if (!isfinite(yaw_deg) || !isfinite(target_deg)) return false;
  const float delta = angle_delta_deg(target_deg, yaw_deg);
  if (fabsf(delta) <= tol_deg) return false;
  if (out_cmd) {
    *out_cmd = {(delta > 0.0f) ? "ROTATE_LEFT" : "ROTATE_RIGHT", 100};
  }
  return true;
}

static float lidar_sector_min_mm(const float bins[360], int start_deg, int end_deg, uint8_t* out_count_close, float close_threshold_mm) {
  float min_mm = NAN;
  uint8_t close_count = 0;
  int d = start_deg;
  while (true) {
    const float v = bins[(d + 360) % 360];
    if (isfinite(v) && v > 0.0f) {
      if (!isfinite(min_mm) || v < min_mm) {
        min_mm = v;
      }
      if (v < close_threshold_mm) {
        close_count++;
      }
    }
    if (d == end_deg) break;
    d = (d + 1) % 360;
  }
  if (out_count_close) *out_count_close = close_count;
  return min_mm;
}

static inline long us_or_inf(long mm) {
  return (mm <= 0) ? 99999 : mm;
}

static double haversine_m(double lat1, double lng1, double lat2, double lng2) {
  if (!isfinite(lat1) || !isfinite(lng1) || !isfinite(lat2) || !isfinite(lng2)) return NAN;
  const double phi1 = lat1 * kDegToRad;
  const double phi2 = lat2 * kDegToRad;
  const double dphi = (lat2 - lat1) * kDegToRad;
  const double dlambda = (lng2 - lng1) * kDegToRad;
  const double a = sin(dphi / 2.0) * sin(dphi / 2.0) +
                   cos(phi1) * cos(phi2) * sin(dlambda / 2.0) * sin(dlambda / 2.0);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return kEarthRadiusM * c;
}

static double bearing_deg(double lat1, double lng1, double lat2, double lng2) {
  if (!isfinite(lat1) || !isfinite(lng1) || !isfinite(lat2) || !isfinite(lng2)) return NAN;
  const double phi1 = lat1 * kDegToRad;
  const double phi2 = lat2 * kDegToRad;
  const double dlambda = (lng2 - lng1) * kDegToRad;
  const double y = sin(dlambda) * cos(phi2);
  const double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dlambda);
  const double b = atan2(y, x) * kRadToDeg;
  const double w = fmod((b + 360.0), 360.0);
  return w;
}

static const char* phase_name(NavPhase phase) {
  switch (phase) {
    case NavPhase::BOOT_CHECK: return "BOOT_CHECK";
    case NavPhase::IDLE: return "IDLE";
    case NavPhase::GOTO_H_INIT: return "GOTO_H_INIT";
    case NavPhase::GOTO_H_LEG1_DRIVE: return "GOTO_H_LEG1_DRIVE";
    case NavPhase::GOTO_H_TURN1: return "GOTO_H_TURN1";
    case NavPhase::GOTO_H_ALIGN1: return "GOTO_H_ALIGN1";
    case NavPhase::GOTO_H_LEG2_DRIVE: return "GOTO_H_LEG2_DRIVE";
    case NavPhase::GOTO_H_TURN2: return "GOTO_H_TURN2";
    case NavPhase::GOTO_H_WAIT_WALL2: return "GOTO_H_WAIT_WALL2";
    case NavPhase::GOTO_H_FIND_WALL2: return "GOTO_H_FIND_WALL2";
    case NavPhase::GOTO_H_ALIGN2: return "GOTO_H_ALIGN2";
    case NavPhase::GOTO_H_LEG3_DRIVE: return "GOTO_H_LEG3_DRIVE";
    case NavPhase::GOTO_H_LEG3_OPEN_DRIVE: return "GOTO_H_LEG3_OPEN_DRIVE";
    case NavPhase::GOTO_H_TURN3: return "GOTO_H_TURN3";
    case NavPhase::GOTO_H_ALIGN3: return "GOTO_H_ALIGN3";
    case NavPhase::GOTO_H_LEG4_DRIVE: return "GOTO_H_LEG4_DRIVE";
    case NavPhase::GOTO_H_TURN4: return "GOTO_H_TURN4";
    case NavPhase::GOTO_H_ALIGN4: return "GOTO_H_ALIGN4";
    case NavPhase::GOTO_H_FINAL_APP: return "GOTO_H_FINAL_APP";
    case NavPhase::GOTO_H_DONE: return "GOTO_H_DONE";
    default: return "UNKNOWN";
  }
}

static String format_status_line(const MotionCmd& cmd, const AppState& sensors) {
  const String action = (cmd.decision && cmd.decision[0]) ? String(cmd.decision) : String("STOP");

  String yaw_txt = "yaw: n/a";
  if (sensors.imu.valid && isfinite(sensors.imu.yaw)) {
    const int yaw_deg = (int)lroundf(wrap_deg(sensors.imu.yaw));
    yaw_txt = "yaw: " + String(yaw_deg);
  }

  return action + " | " + yaw_txt;
}

static String format_us_line(const USBlock& us) {
  return String("F:") + String(us.front_mm) +
         " R:" + String(us.right_mm) +
         " L:" + String(us.left_mm);
}

struct ObstacleCtx {
   ObsState state = ObsState::CHECK;
   uint32_t state_start_ms = 0;

   // For Static Analysis
   float snapshot_bins[360]; 
   bool snapshot_taken = false;

   // For Return-to-Route Logic
   bool initial_step_was_right = false; 
   uint32_t accumulated_step_ms = 0;
   uint32_t last_step_tick_ms = 0;
};
static ObstacleCtx g_obstacle;

struct MissionCtx {
  NavPhase phase = NavPhase::BOOT_CHECK;
  uint32_t phase_transition_end_ms = 0;

  bool turn_target_set = false;
  float turn_target_yaw = NAN;
  uint32_t turn_started_ms = 0;
  bool drive_aim_set = false;
  float drive_aim_yaw = NAN;

  bool leg2_wall_seen = false;
  bool leg2_realign_active = false;
  uint32_t leg2_wall_lost_ms = 0;
  uint32_t correction_until_ms = 0;
  MotionCmd correction_cmd = {};

  bool leg1_approach_latched = false;
  bool leg1_benches_seen = false;
  bool leg1_benches_passed = false;
  bool leg1_left_seen = false;
  bool leg1_left_dropped = false;
  bool leg3_approach_latched = false;
  bool leg3_realign_active = false;
};

static MissionCtx g_mission;
static Kalman2D g_gps_kf;
static uint8_t g_route_seq = 0;

static inline void mission_transition(NavPhase next, uint32_t now_ms, bool pause = true) {
  g_mission.phase = next;
  g_mission.phase_transition_end_ms = pause ? (now_ms + kPhaseTransitionPauseMs) : now_ms;
  g_mission.turn_target_set = false;
  g_mission.turn_target_yaw = NAN;
  g_mission.turn_started_ms = 0;
  g_mission.correction_until_ms = 0;
  g_mission.correction_cmd = {};
  g_mission.leg2_wall_lost_ms = 0;
  if (next == NavPhase::BOOT_CHECK || next == NavPhase::IDLE || next == NavPhase::GOTO_H_INIT) {
    g_mission.drive_aim_set = false;
    g_mission.drive_aim_yaw = NAN;
  }
  if (next == NavPhase::GOTO_H_LEG2_DRIVE) {
    g_mission.leg2_wall_seen = false;
    g_mission.leg2_realign_active = false;
  }
  if (next == NavPhase::GOTO_H_LEG1_DRIVE) {
    g_mission.leg1_approach_latched = false;
    g_mission.leg1_benches_seen = false;
    g_mission.leg1_benches_passed = false;
    g_mission.leg1_left_seen = false;
    g_mission.leg1_left_dropped = false;
  }
  if (next == NavPhase::GOTO_H_LEG3_DRIVE) {
    g_mission.leg3_realign_active = false;
  }
  if (next == NavPhase::GOTO_H_LEG3_OPEN_DRIVE) {
    g_mission.leg3_approach_latched = false;
  }
}

static inline void mission_reset(NavPhase phase, uint32_t now_ms) {
  g_mission = {};
  g_mission.phase = phase;
  g_mission.phase_transition_end_ms = now_ms + kPhaseTransitionPauseMs;
  g_obstacle = {};
}

static inline void set_drive_aim_from_turn() {
  if (g_mission.turn_target_set && isfinite(g_mission.turn_target_yaw)) {
    g_mission.drive_aim_yaw = g_mission.turn_target_yaw;
    g_mission.drive_aim_set = true;
  }
}

static inline bool drive_align_needed(const AppState& sensors, float yaw, MotionCmd* out_cmd) {
  if (!g_mission.drive_aim_set || !sensors.imu.valid) return false;
  float biased_target = wrap_deg(g_mission.drive_aim_yaw + 16.0f); // QUICK FIX FOR LEFT SHIFT ISSUE
  return drive_yaw_align_cmd(yaw, g_mission.drive_aim_yaw, kDriveYawTolDeg, out_cmd);
}

static void apply_us_schedule_for_phase(NavPhase phase) {
  switch (phase) {
    case NavPhase::GOTO_H_INIT:
      sensor_us_set_active(true, false, true, true, false);
      break;
    case NavPhase::GOTO_H_LEG1_DRIVE:
      sensor_us_set_active(true, false, true, false, false);
      break;
    case NavPhase::GOTO_H_TURN1:
      sensor_us_set_active(false, false, true, true, false);
      break;
    case NavPhase::GOTO_H_ALIGN1:
    case NavPhase::GOTO_H_LEG2_DRIVE:
    case NavPhase::GOTO_H_TURN2:
    case NavPhase::GOTO_H_WAIT_WALL2:
    case NavPhase::GOTO_H_FIND_WALL2:
    case NavPhase::GOTO_H_ALIGN2:
      sensor_us_set_active(false, false, false, true, false);
      break;
    case NavPhase::GOTO_H_LEG3_DRIVE:
      sensor_us_set_active(false, false, false, true, false);
      break;
    case NavPhase::GOTO_H_LEG3_OPEN_DRIVE:
      sensor_us_set_active(true, false, false, false, false);
      break;
    case NavPhase::GOTO_H_TURN3:
    case NavPhase::GOTO_H_ALIGN3:
      sensor_us_set_active(false, false, true, false, false);
      break;
    case NavPhase::GOTO_H_LEG4_DRIVE:
      sensor_us_set_active(true, false, true, false, false);
      break;
    case NavPhase::GOTO_H_TURN4:
    case NavPhase::GOTO_H_ALIGN4:
      sensor_us_set_active(false, false, true, false, false);
      break;
    case NavPhase::GOTO_H_FINAL_APP:
      sensor_us_set_active(false, false, true, true, false);
      break;
    case NavPhase::GOTO_H_DONE:
    case NavPhase::BOOT_CHECK:
    case NavPhase::IDLE:
    default:
      sensor_us_set_active(true, false, true, true, false);
      break;
  }
}

struct ClusterStats {
  int count = 0;
  int start_scan = 0;
  int end_scan = 0;
  float sum_dist = 0.0f;
  float min_dist = NAN;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_yy = 0.0;
  double sum_xy = 0.0;
};

static ObstacleType classify_cluster(const ClusterStats& cluster) {
  if (cluster.count < kObstacleClusterMinPoints) return ObstacleType::None;

  const int span_deg = cluster.end_scan - cluster.start_scan;
  if (span_deg > (int)kObstacleWallSpanDeg) return ObstacleType::Wall;

  const double inv = 1.0 / (double)cluster.count;
  const double mean_x = cluster.sum_x * inv;
  const double mean_y = cluster.sum_y * inv;
  double var_x = cluster.sum_xx * inv - mean_x * mean_x;
  double var_y = cluster.sum_yy * inv - mean_y * mean_y;
  double cov_xy = cluster.sum_xy * inv - mean_x * mean_y;
  if (var_x < 0.0) var_x = 0.0;
  if (var_y < 0.0) var_y = 0.0;
  const double trace = var_x + var_y;
  const double det = var_x * var_y - cov_xy * cov_xy;
  double disc = trace * trace - 4.0 * det;
  if (disc < 0.0) disc = 0.0;
  const double sqrt_disc = sqrt(disc);
  const double lambda1 = 0.5 * (trace + sqrt_disc);
  const double lambda2 = 0.5 * (trace - sqrt_disc);
  if (lambda1 <= 1e-3) return ObstacleType::Wall;
  if ((lambda2 / lambda1) <= (double)kObstacleLineRatio) return ObstacleType::Wall;
  return ObstacleType::Unknown;
}

static float detect_obstacle_cluster(const float bins[360],
                                     int start_deg,
                                     int end_deg,
                                     float min_dist_mm,
                                     float max_dist_mm,
                                     ObstacleType& out_type) {
  out_type = ObstacleType::None;
  if (!bins) return NAN;

  float closest_avg = NAN;
  ObstacleType closest_type = ObstacleType::None;

  ClusterStats cluster = {};
  bool in_cluster = false;
  float prev_dist = NAN;

  auto finalize_cluster = [&](const ClusterStats& candidate) {
    const ObstacleType type = classify_cluster(candidate);
    if (type == ObstacleType::None) return;
    const float avg_dist = candidate.sum_dist / (float)candidate.count;
    if (!isfinite(closest_avg) || avg_dist < closest_avg) {
      closest_avg = avg_dist;
      closest_type = type;
    }
  };

  const int start = wrap_deg_i(start_deg);
  const int end = wrap_deg_i(end_deg);
  int scan_idx = 0;
  int d = start;
  while (true) {
    const float dist = bins[d];
    const bool valid = isfinite(dist) && (dist >= min_dist_mm) && (dist <= max_dist_mm);

    if (!valid) {
      if (in_cluster) {
        finalize_cluster(cluster);
      }
      in_cluster = false;
      prev_dist = NAN;
    } else {
      const bool gap = isfinite(prev_dist) && (fabsf(dist - prev_dist) > kObstacleClusterGapMm);
      if (!in_cluster || gap) {
        if (in_cluster) {
          finalize_cluster(cluster);
        }
        cluster = {};
        cluster.start_scan = scan_idx;
        cluster.end_scan = scan_idx;
        cluster.min_dist = dist;
        in_cluster = true;
      }

      const double rad = (double)d * kDegToRad;
      const double x = (double)dist * cos(rad);
      const double y = (double)dist * sin(rad);

      cluster.count++;
      cluster.sum_dist += dist;
      if (!isfinite(cluster.min_dist) || dist < cluster.min_dist) cluster.min_dist = dist;
      cluster.end_scan = scan_idx;
      cluster.sum_x += x;
      cluster.sum_y += y;
      cluster.sum_xx += x * x;
      cluster.sum_yy += y * y;
      cluster.sum_xy += x * y;
      prev_dist = dist;
    }

    if (d == end) {
      if (in_cluster) {
        finalize_cluster(cluster);
      }
      break;
    }
    d = (d + 1) % 360;
    scan_idx++;
    if (scan_idx > 360) break;
  }

  if (isfinite(closest_avg)) {
    out_type = closest_type;
    return closest_avg;
  }
  return NAN;
}

static bool choose_step_right(const AppState& sensors) {
  ObstacleType left_type = ObstacleType::None;
  ObstacleType right_type = ObstacleType::None;
  const float left_dist =
      detect_obstacle_cluster(sensors.lidarBins, 60, 120, kObstacleMinDistMm, kObstacleMaxDistMm, left_type);
  const float right_dist =
      detect_obstacle_cluster(sensors.lidarBins, 240, 300, kObstacleMinDistMm, kObstacleMaxDistMm, right_type);

  const float left_score = isfinite(left_dist) ? left_dist : kObstacleClearMm;
  const float right_score = isfinite(right_dist) ? right_dist : kObstacleClearMm;

  if (fabsf(left_score - right_score) < 50.0f) {
    return true;
  }
  return right_score >= left_score;
}

static MotionCmd obstacle_tick(const AppState& sensors, uint32_t now_ms, const MotionCmd& planned, bool* out_overridden) {
  if (out_overridden) *out_overridden = false;

  // 1. Detection & Trigger Logic
  // Check front sector +/- 20 degrees (340 to 20) with strict 600mm threshold
  uint8_t c = 0;
  const float lidar_front_min = lidar_sector_min_mm(sensors.lidarBins, 340, 20, &c, 600.0f);
  const bool lidar_trigger = (isfinite(lidar_front_min) && lidar_front_min < 600.0f);
  
  const long us_front = sensors.us.front_mm;
  // Ignore 0 values on US as noise/max-range, only trigger on positive valid readings < 600
  const bool us_trigger = (us_front > 0 && us_front < 600);

  if ((lidar_trigger || us_trigger) && g_obstacle.state == ObsState::CHECK) {
     g_obstacle.state = ObsState::ASSESS_TYPE;
     g_obstacle.state_start_ms = now_ms;
     g_obstacle.snapshot_taken = false;
     // Immediately override
     if (out_overridden) *out_overridden = true;
     return {"STOP", 0};
  }

  // Helper check for "CLEAR" condition
  // lidarBins[350..10] > 4000mm AND sensors.us.front_mm > 2500
  auto is_front_clear = [&]() {
      uint8_t c_inner = 0;
      float fm = lidar_sector_min_mm(sensors.lidarBins, 350, 10, &c_inner, 4000.0f); 
      bool l_clear = (!isfinite(fm) || fm > 4000.0f);
      bool u_clear = (sensors.us.front_mm == 0 || sensors.us.front_mm > 2500); 
      return l_clear && u_clear;
  };

  switch (g_obstacle.state) {
    case ObsState::CHECK:
      return planned;

    case ObsState::ASSESS_TYPE: {
      if (out_overridden) *out_overridden = true;
      
      // Capture snapshot on first entry
      if (!g_obstacle.snapshot_taken) {
         memcpy(g_obstacle.snapshot_bins, sensors.lidarBins, sizeof(sensors.lidarBins));
         g_obstacle.snapshot_taken = true;
         // Wait a brief moment to accumulate variance? Or just use one frame?
         // Prompt says: "While stopped, compare current LiDAR readings against a snapshot... If readings fluctuate significantly..."
         return {"STOP", 0};
      }

      // Assess duration - give it 500ms to see if things move
      if ((now_ms - g_obstacle.state_start_ms) < 500) {
         return {"STOP", 0};
      }

      // Compare current vs snapshot
      float total_diff = 0;
      int valid_pts = 0;
      for(int i=0; i<360; ++i) {
         float curr = sensors.lidarBins[i];
         float snap = g_obstacle.snapshot_bins[i];
         if (isfinite(curr) && isfinite(snap) && curr > 0 && snap > 0) {
             total_diff += fabsf(curr - snap);
             valid_pts++;
         }
      }
      
      // Tolerance check
      bool is_dynamic = false;
      if (valid_pts > 0) {
          float avg_diff = total_diff / valid_pts;
          if (avg_diff > 200.0f) { // Tolerance > 200mm avg change
              is_dynamic = true;
          }
      }

      if (is_dynamic) {
          g_obstacle.state = ObsState::WAIT_DYNAMIC;
      } else {
          // STATIC -> Step Out
          g_obstacle.state = ObsState::AVOID_STEP_OUT;
          
          // Determine Direction: check edges relative to 0 (Front)
          // Scan Left (0 to 90) and Right (360 down to 270) to find "end" of obstacle
          // Actually, "Left" is positive deg (0..90..180)? Standard math: CCW is positive.
          // In this system: 350..10 is Front. 90 is Left, 270 is Right.
          // Let's check which side has closer edge edge.
          // Simple heuristic: Sum/Avg distance in Left Sector (20-90) vs Right Sector (270-340)
          // "Choose the direction ... that requires the least travel" -> path of least resistance.
          
          uint8_t c_l=0, c_r=0;
          float min_l = lidar_sector_min_mm(sensors.lidarBins, 20, 90, &c_l, 4000.0f);
          float min_r = lidar_sector_min_mm(sensors.lidarBins, 270, 340, &c_r, 4000.0f);
          
          // If Right side is more open (larger min dist) -> Step Right? 
          // Usually we want to step TOWARDS the open space.
          // If min_r > min_l, Right is clearer.
          float score_l = isfinite(min_l) ? min_l : 5000.0f;
          float score_r = isfinite(min_r) ? min_r : 5000.0f;
          
          g_obstacle.initial_step_was_right = (score_r >= score_l);
          
          g_obstacle.accumulated_step_ms = 0;
          g_obstacle.last_step_tick_ms = now_ms;
      }
      return {"STOP", 0};
    }

    case ObsState::WAIT_DYNAMIC:
      if (out_overridden) *out_overridden = true;
      if (is_front_clear()) {
         g_obstacle.state = ObsState::CLEAR;
         // Resume planned
         return planned;
      }
      return {"STOP", 0};

    case ObsState::AVOID_STEP_OUT: {
      if (out_overridden) *out_overridden = true;
      
      // Accumulate time
      uint32_t dt = now_ms - g_obstacle.last_step_tick_ms;
      g_obstacle.accumulated_step_ms += dt;
      g_obstacle.last_step_tick_ms = now_ms;

      // Check clearance
      if (is_front_clear()) {
          g_obstacle.state = ObsState::AVOID_FORWARD;
          return {"STOP", 0}; // Transition
      }
      
      return {g_obstacle.initial_step_was_right ? "STEP_RIGHT" : "STEP_LEFT", 100};
    }

    case ObsState::AVOID_FORWARD: {
      if (out_overridden) *out_overridden = true;
      
      // Check Side Clearance relative to obstacle
      // If we stepped RIGHT, obstacle is to our LEFT. Check Left Side (e.g. 70-110 deg).
      // If we stepped LEFT, obstacle is to our RIGHT. Check Right Side (e.g. 250-290 deg).
      
      bool side_clear = false;
      uint8_t c_side = 0;
      float side_min = NAN;
      long us_side = 0;

      if (g_obstacle.initial_step_was_right) {
         // Check Left
         side_min = lidar_sector_min_mm(sensors.lidarBins, 70, 110, &c_side, 2000.0f);
         us_side = sensors.us.left_mm;
      } else {
         // Check Right
         side_min = lidar_sector_min_mm(sensors.lidarBins, 250, 290, &c_side, 2000.0f);
         us_side = sensors.us.right_mm;
      }
      
      // "Once the side is clear (meaning we passed the obstacle length)"
      // Threshold: > 1500mm or so? 
      // User didn't specify exact side clearance threshold, but "clear (> 4000mm)" was for front.
      // Let's use > 1500mm as "Clear enough" for side passing, or inf.
      bool l_clr = (!isfinite(side_min) || side_min > 1200.0f);
      bool u_clr = (us_side == 0 || us_side > 1200);

      if (l_clr && u_clr) {
          g_obstacle.state = ObsState::AVOID_STEP_IN;
          g_obstacle.last_step_tick_ms = now_ms;
          return {"STOP", 0};
      }
      
      return {"FORWARD", 100};
    }

    case ObsState::AVOID_STEP_IN: {
      if (out_overridden) *out_overridden = true;
      
      // Calculate remaining time
      uint32_t dt = now_ms - g_obstacle.last_step_tick_ms;
      
      if (g_obstacle.accumulated_step_ms > dt) {
          g_obstacle.accumulated_step_ms -= dt;
      } else {
          g_obstacle.accumulated_step_ms = 0;
      }
      g_obstacle.last_step_tick_ms = now_ms;

      if (g_obstacle.accumulated_step_ms == 0) {
          g_obstacle.state = ObsState::CLEAR;
          return {"STOP", 0}; // Done
      }

      // Step Anti-X
      return {g_obstacle.initial_step_was_right ? "STEP_LEFT" : "STEP_RIGHT", 100};
    }

    case ObsState::CLEAR:
      g_obstacle = {};
      return planned;
  }

  return planned;
}

static void update_obstacle_snapshot(const AppState& sensors, ObstacleSnapshot& out) {
  out = {};
  out.possible_obstacles = "";
  out.closest_dist_mm = NAN;

  uint8_t c = 0;
  const float f = lidar_sector_min_mm(sensors.lidarBins, 350, 10, &c, kLidarObstacleMm);
  if (isfinite(f) && f < kLidarObstacleMm && c >= kLidarObstacleMinBins) out.front_blocked = true;
  const float l = lidar_sector_min_mm(sensors.lidarBins, 70, 110, &c, kLidarObstacleMm);
  if (isfinite(l) && l < kLidarObstacleMm && c >= kLidarObstacleMinBins) out.left_blocked = true;
  const float r = lidar_sector_min_mm(sensors.lidarBins, 250, 290, &c, kLidarObstacleMm);
  if (isfinite(r) && r < kLidarObstacleMm && c >= kLidarObstacleMinBins) out.right_blocked = true;
  const float b = lidar_sector_min_mm(sensors.lidarBins, 170, 190, &c, kLidarObstacleMm);
  if (isfinite(b) && b < kLidarObstacleMm && c >= kLidarObstacleMinBins) out.rear_blocked = true;

  float min_any = f;
  if (isfinite(l) && (!isfinite(min_any) || l < min_any)) min_any = l;
  if (isfinite(r) && (!isfinite(min_any) || r < min_any)) min_any = r;
  if (isfinite(b) && (!isfinite(min_any) || b < min_any)) min_any = b;
  out.closest_dist_mm = min_any;
  out.hard_stop = isfinite(min_any) && min_any < kLidarHardStopMm;
}

static MotionCmd goto_h_tick(const AppState& sensors, uint32_t now_ms) {
  const long front = us_or_inf(sensors.us.front_mm);
  const long left = us_or_inf(sensors.us.left_mm);
  const long right = us_or_inf(sensors.us.right_mm);
  const float yaw = sensors.imu.yaw;
  const bool gps_trust = sensors.gps.valid && isfinite(sensors.gps.hdop) &&
                         (sensors.gps.hdop < kGpsTrustMaxHdop) && (sensors.gps.sats >= kGpsTrustMinSats);

  apply_us_schedule_for_phase(g_mission.phase);

  if (now_ms < g_mission.phase_transition_end_ms) {
    return {"STOP", 0};
  }
  const uint32_t phase_run_ms = now_ms - g_mission.phase_transition_end_ms;

  switch (g_mission.phase) {
    case NavPhase::GOTO_H_INIT: {
      uint8_t close_count = 0;
      const float min_front = lidar_sector_min_mm(sensors.lidarBins, 350, 10, &close_count, kLidarClearanceInitMm);
      const bool clear = !isfinite(min_front) || (min_front >= kLidarClearanceInitMm) || (close_count == 0);
      if (!clear) {
        return {"STOP", 0};
      }
      if (phase_run_ms < kInitSlowMs) {
        return {"FORWARD", 100};
      }
      mission_transition(NavPhase::GOTO_H_LEG1_DRIVE, now_ms, /*pause=*/false);
      return {"FORWARD", 100};
    }

    case NavPhase::GOTO_H_LEG1_DRIVE: {
      g_nav.dest_lat = kLeg1TargetLat;
      g_nav.dest_lng = kLeg1TargetLng;
      // GPS/Bearing calculations kept for telemetry
      if (g_nav.ref_valid) {
        g_nav.distance_to_dest_m = (float)haversine_m(g_nav.ref_lat, g_nav.ref_lng, kLeg1TargetLat, kLeg1TargetLng);
        g_nav.bearing_to_dest_deg = (float)bearing_deg(g_nav.ref_lat, g_nav.ref_lng, kLeg1TargetLat, kLeg1TargetLng);
      }
      
      const float yaw = sensors.imu.yaw;
      if (!g_mission.drive_aim_set && sensors.imu.valid && isfinite(yaw)) {
        g_mission.drive_aim_yaw = wrap_deg(yaw);
        g_mission.drive_aim_set = true;
      }

      const long front = sensors.us.front_mm;
      const long left = sensors.us.left_mm;
      const long right = sensors.us.right_mm;
      
      // Phase 1: Benches Detection (Left & Right both valid < 2500)
      if (!g_mission.leg1_benches_seen) {
         if (left > 0 && left < 2500 && right > 0 && right < 2500) {
             g_mission.leg1_benches_seen = true;
         }
      } 
      // Phase 1b: Benches Passed (Left & Right both invalid or > 2500)
      else if (!g_mission.leg1_benches_passed) {
         bool l_clear = (left <= 0 || left > 2500);
         bool r_clear = (right <= 0 || right > 2500);
         if (l_clear && r_clear) {
             g_mission.leg1_benches_passed = true;
         }
      }

      // Phase 2: Final Marker (Left detected then lost/passed)
      // Only check this AFTER benches are passed
      if (g_mission.leg1_benches_passed) {
          if (!g_mission.leg1_left_seen) {
             if (left > 0 && left < 2500) {
                 g_mission.leg1_left_seen = true;
             }
          } else if (!g_mission.leg1_left_dropped) {
             bool l_clear = (left <= 0 || left > 2500);
             if (l_clear) {
                 g_mission.leg1_left_dropped = true;
             }
          }
      }

      // Phase 3: Final Wall Stop
      // Only check this AFTER final marker logic is done
      if (g_mission.leg1_left_dropped) {
          if (front > 0 && front < 1500) {
            mission_transition(NavPhase::GOTO_H_TURN1, now_ms);
            return {"STOP", 0};
          }
      }

      MotionCmd yaw_cmd = {};
      if (drive_align_needed(sensors, yaw, &yaw_cmd)) return yaw_cmd;
      return {"FORWARD", 100};
    }

    case NavPhase::GOTO_H_TURN1: {
      if (!isfinite(yaw)) return {"STOP", 0};
      if (!g_mission.turn_target_set) {
        g_mission.turn_target_set = true;
        g_mission.turn_target_yaw = wrap_deg(yaw - 90.0f);
        g_mission.turn_started_ms = now_ms;
      }
      if (aligned_yaw(yaw, g_mission.turn_target_yaw, kYawTolDeg)) {
        set_drive_aim_from_turn();
        mission_transition(NavPhase::GOTO_H_ALIGN1, now_ms);
        return {"STOP", 0};
      }
      if ((now_ms - g_mission.turn_started_ms) > kTurnTimeoutMs) {
        set_drive_aim_from_turn();
        mission_transition(NavPhase::GOTO_H_ALIGN1, now_ms);
        return {"STOP", 0};
      }
      return {"ROTATE_LEFT", 100};
    }

    case NavPhase::GOTO_H_ALIGN1: {
      if (right < (kWallTargetMm - kWallTolMm)) return {"STEP_LEFT", 100};
      if (right > (kWallTargetMm + kWallTolMm)) return {"STEP_RIGHT", 100};
      mission_transition(NavPhase::GOTO_H_LEG2_DRIVE, now_ms);
      return {"STOP", 0};
    }

    case NavPhase::GOTO_H_LEG2_DRIVE: {
      g_nav.dest_lat = kLeg2TargetLat;
      g_nav.dest_lng = kLeg2TargetLng;
      if (g_nav.ref_valid) {
        const double dist_m = haversine_m(g_nav.ref_lat, g_nav.ref_lng, kLeg2TargetLat, kLeg2TargetLng);
        g_nav.distance_to_dest_m = (float)dist_m;
        g_nav.bearing_to_dest_deg = (float)bearing_deg(g_nav.ref_lat, g_nav.ref_lng, kLeg2TargetLat, kLeg2TargetLng);
      }

      const long right_raw = sensors.us.right_mm;
      const bool right_valid = right_raw > 0;
      const bool no_wall = !right_valid || (right > kNoWallMm);
      const bool wall_visible = right_valid && !no_wall;

      if (wall_visible) {
        g_mission.leg2_wall_seen = true;
      }

      if (g_mission.leg2_wall_seen) {
        if (!right_valid) {
          g_mission.leg2_realign_active = false;
          if (!g_mission.leg2_wall_lost_ms) {
            g_mission.leg2_wall_lost_ms = now_ms;
          }
          if ((now_ms - g_mission.leg2_wall_lost_ms) >= 1000) {
            mission_transition(NavPhase::GOTO_H_TURN2, now_ms);
            return {"STOP", 0};
          }
          MotionCmd yaw_cmd = {};
          if (drive_align_needed(sensors, yaw, &yaw_cmd)) return yaw_cmd;
          return {"FORWARD", 100};
        }

        g_mission.leg2_wall_lost_ms = 0;
        if (right > kNoWallMm) {
          g_mission.leg2_realign_active = false;
          mission_transition(NavPhase::GOTO_H_TURN2, now_ms);
          return {"STOP", 0};
        }
      } else {
        g_mission.leg2_wall_lost_ms = 0;
      }

      // Suppress repeated micro-corrections while driving. Only re-align if we get dangerously close.
      if (!g_mission.leg2_realign_active && wall_visible && (right_raw < kDriveRealignTriggerMm)) {
        g_mission.leg2_realign_active = true;
      }
      if (g_mission.leg2_realign_active) {
        if (!right_valid) {
          g_mission.leg2_realign_active = false;
        } else if (right < (kWallTargetMm - kWallTolMm)) {
          return {"STEP_LEFT", 100};
        } else if (right > (kWallTargetMm + kWallTolMm)) {
          return {"STEP_RIGHT", 100};
        } else {
          g_mission.leg2_realign_active = false;
        }
      }
      MotionCmd yaw_cmd = {};
      if (drive_align_needed(sensors, yaw, &yaw_cmd)) return yaw_cmd;
      return {"FORWARD", 100};
    }

    case NavPhase::GOTO_H_TURN2: {
      if (!isfinite(yaw)) return {"STOP", 0};
      if (!g_mission.turn_target_set) {
        g_mission.turn_target_set = true;
        g_mission.turn_target_yaw = wrap_deg(yaw + 90.0f);
        g_mission.turn_started_ms = now_ms;
      }
      if (aligned_yaw(yaw, g_mission.turn_target_yaw, kYawTolDeg)) {
        set_drive_aim_from_turn();
        mission_transition(NavPhase::GOTO_H_WAIT_WALL2, now_ms);
        return {"STOP", 0};
      }
      if ((now_ms - g_mission.turn_started_ms) > kTurnTimeoutMs) {
        set_drive_aim_from_turn();
        mission_transition(NavPhase::GOTO_H_WAIT_WALL2, now_ms);
        return {"STOP", 0};
      }
      return {"ROTATE_RIGHT", 100};
    }



    case NavPhase::GOTO_H_WAIT_WALL2: {
      const long r = sensors.us.right_mm;
      

      bool wall_found = (r > 100) && (r < 2500);

      if (phase_run_ms > 6000) {
           mission_transition(NavPhase::GOTO_H_ALIGN2, now_ms);
           return {"STOP", 0};
      }

      if (wall_found) {
        mission_transition(NavPhase::GOTO_H_ALIGN2, now_ms);
        return {"STOP", 0};
      }

      return {"FORWARD", 100};
    }

    case NavPhase::GOTO_H_ALIGN2: {
      if (right < (kWallTargetMm - kWallTolMm)) return {"STEP_LEFT", 100};
      if (right > (kWallTargetMm + kWallTolMm)) return {"STEP_RIGHT", 100};
      mission_transition(NavPhase::GOTO_H_LEG3_DRIVE, now_ms);
      return {"STOP", 0};
    }

    case NavPhase::GOTO_H_LEG3_DRIVE: {
      g_nav.dest_lat = kLeg3TargetLat;
      g_nav.dest_lng = kLeg3TargetLng;
      if (g_nav.ref_valid) {
        const double dist_m = haversine_m(g_nav.ref_lat, g_nav.ref_lng, kLeg3TargetLat, kLeg3TargetLng);
        g_nav.distance_to_dest_m = (float)dist_m;
        g_nav.bearing_to_dest_deg = (float)bearing_deg(g_nav.ref_lat, g_nav.ref_lng, kLeg3TargetLat, kLeg3TargetLng);
      }

      const long right_raw = sensors.us.right_mm;
      const bool right_valid = right_raw > 0;
      if (!right_valid) {
        g_mission.leg3_realign_active = false;
        mission_transition(NavPhase::GOTO_H_LEG3_OPEN_DRIVE, now_ms, /*pause=*/false);
        MotionCmd yaw_cmd = {};
        if (drive_align_needed(sensors, yaw, &yaw_cmd)) return yaw_cmd;
        return {"FORWARD", 100};
      }

      // Same "only when too close" re-align gating as LEG2.
      if (!g_mission.leg3_realign_active && (right_raw < kDriveRealignTriggerMm)) {
        g_mission.leg3_realign_active = true;
      }
      if (g_mission.leg3_realign_active) {
        if (right < (kLeg3WallTargetMm - kWallTolMm)) {
          return {"STEP_LEFT", 100};
        }
        if (right > (kLeg3WallTargetMm + kWallTolMm)) {
          return {"STEP_RIGHT", 100};
        }
        g_mission.leg3_realign_active = false;
      }

      MotionCmd yaw_cmd = {};
      if (drive_align_needed(sensors, yaw, &yaw_cmd)) return yaw_cmd;
      return {"FORWARD", 100};
    }

    case NavPhase::GOTO_H_LEG3_OPEN_DRIVE: {
      g_nav.dest_lat = kLeg3TargetLat;
      g_nav.dest_lng = kLeg3TargetLng;
      double dist_m = NAN;
      if (g_nav.ref_valid) {
        dist_m = haversine_m(g_nav.ref_lat, g_nav.ref_lng, kLeg3TargetLat, kLeg3TargetLng);
        g_nav.distance_to_dest_m = (float)dist_m;
        g_nav.bearing_to_dest_deg = (float)bearing_deg(g_nav.ref_lat, g_nav.ref_lng, kLeg3TargetLat, kLeg3TargetLng);
      }

      const long front_raw = sensors.us.front_mm;
      const bool gps_ok = gps_trust && isfinite(dist_m);
      const bool gps_near = gps_ok && (dist_m <= (double)kLeg3GpsProximityM);

      const long front_delta = front_raw - kWallTargetMm;
      const bool us_in_turn_band =
          (front_raw > 0) && (front_delta >= -kWallTolMm) && (front_delta <= kWallTolMm);

      if (!g_mission.leg3_approach_latched && (gps_near || !gps_ok)) {
        g_mission.leg3_approach_latched = true;
      }

      // Once we've entered approach (distance threshold or GPS health drop), stay in approach and
      // rely only on the front ultrasonic band for the TURN3 trigger.
      if (!g_mission.leg3_approach_latched) {
        MotionCmd yaw_cmd = {};
        if (drive_align_needed(sensors, yaw, &yaw_cmd)) return yaw_cmd;
        return {"FORWARD", 100};
      }
      if (us_in_turn_band) {
        mission_transition(NavPhase::GOTO_H_TURN3, now_ms);
        return {"STOP", 0};
      }
      MotionCmd yaw_cmd = {};
      if (drive_align_needed(sensors, yaw, &yaw_cmd)) return yaw_cmd;
      return {"FORWARD", 100};
    }

    case NavPhase::GOTO_H_TURN3: {
      if (!isfinite(yaw)) return {"STOP", 0};
      if (!g_mission.turn_target_set) {
        g_mission.turn_target_set = true;
        g_mission.turn_target_yaw = wrap_deg(yaw + 90.0f);
        g_mission.turn_started_ms = now_ms;
      }
      if (aligned_yaw(yaw, g_mission.turn_target_yaw, kYawTolDeg)) {
        set_drive_aim_from_turn();
        mission_transition(NavPhase::GOTO_H_ALIGN3, now_ms);
        return {"STOP", 0};
      }
      if ((now_ms - g_mission.turn_started_ms) > kTurnTimeoutMs) {
        set_drive_aim_from_turn();
        mission_transition(NavPhase::GOTO_H_ALIGN3, now_ms);
        return {"STOP", 0};
      }
      return {"ROTATE_RIGHT", 100};
    }

    case NavPhase::GOTO_H_ALIGN3: {
      if (left < (kWallTargetMm - kWallTolMm)) return {"STEP_RIGHT", 100};
      if (left > (kWallTargetMm + kWallTolMm)) return {"STEP_LEFT", 100};
      mission_transition(NavPhase::GOTO_H_LEG4_DRIVE, now_ms);
      return {"STOP", 0};
    }

    case NavPhase::GOTO_H_LEG4_DRIVE: {
      g_nav.dest_lat = kLeg4TargetLat;
      g_nav.dest_lng = kLeg4TargetLng;
      if (g_nav.ref_valid) {
        const double dist_m = haversine_m(g_nav.ref_lat, g_nav.ref_lng, kLeg4TargetLat, kLeg4TargetLng);
        g_nav.distance_to_dest_m = (float)dist_m;
        g_nav.bearing_to_dest_deg = (float)bearing_deg(g_nav.ref_lat, g_nav.ref_lng, kLeg4TargetLat, kLeg4TargetLng);
      }

      if (front < 2000) {
        mission_transition(NavPhase::GOTO_H_TURN4, now_ms);
        return {"STOP", 0};
      }
      if (front < 2500) {
        MotionCmd yaw_cmd = {};
        if (drive_align_needed(sensors, yaw, &yaw_cmd)) return yaw_cmd;
        return {"FORWARD", 100};
      }
      MotionCmd yaw_cmd = {};
      if (drive_align_needed(sensors, yaw, &yaw_cmd)) return yaw_cmd;
      return {"FORWARD", 100};
    }

    case NavPhase::GOTO_H_TURN4: {
      if (!isfinite(yaw)) return {"STOP", 0};
      if (!g_mission.turn_target_set) {
        g_mission.turn_target_set = true;
        g_mission.turn_target_yaw = wrap_deg(yaw + 90.0f);
        g_mission.turn_started_ms = now_ms;
      }
      if (aligned_yaw(yaw, g_mission.turn_target_yaw, kYawTolDeg)) {
        set_drive_aim_from_turn();
        mission_transition(NavPhase::GOTO_H_ALIGN4, now_ms);
        return {"STOP", 0};
      }
      if ((now_ms - g_mission.turn_started_ms) > kTurnTimeoutMs) {
        set_drive_aim_from_turn();
        mission_transition(NavPhase::GOTO_H_ALIGN4, now_ms);
        return {"STOP", 0};
      }
      return {"ROTATE_RIGHT", 100};
    }

    case NavPhase::GOTO_H_ALIGN4: {
      if (left < (kWallWideTargetMm - kWallWideTolMm)) return {"STEP_RIGHT", 100};
      if (left > (kWallWideTargetMm + kWallWideTolMm)) return {"STEP_LEFT", 100};
      mission_transition(NavPhase::GOTO_H_FINAL_APP, now_ms);
      return {"STOP", 0};
    }

    case NavPhase::GOTO_H_FINAL_APP: {
      g_nav.dest_lat = kFinalAppTargetLat;
      g_nav.dest_lng = kFinalAppTargetLng;
      if (g_nav.ref_valid) {
        const double dist_m = haversine_m(g_nav.ref_lat, g_nav.ref_lng, kFinalAppTargetLat, kFinalAppTargetLng);
        g_nav.distance_to_dest_m = (float)dist_m;
        g_nav.bearing_to_dest_deg =
            (float)bearing_deg(g_nav.ref_lat, g_nav.ref_lng, kFinalAppTargetLat, kFinalAppTargetLng);
      }

      const long r = us_or_inf(sensors.us.right_mm);
      if (r < 1000) {
        mission_transition(NavPhase::GOTO_H_DONE, now_ms, /*pause=*/false);
        return {"STOP", 0};
      }

      const long left_raw = sensors.us.left_mm;
      if (left_raw > 0) {
        if (left_raw < (kWallWideTargetMm - kWallWideTolMm)) return {"STEP_RIGHT", 100};
        if (left_raw > (kWallWideTargetMm + kWallWideTolMm)) return {"STEP_LEFT", 100};
      }
      MotionCmd yaw_cmd = {};
      if (drive_align_needed(sensors, yaw, &yaw_cmd)) return yaw_cmd;
      return {"FORWARD", 100};
    }

    case NavPhase::GOTO_H_DONE:
      return {"STOP", 0};

    case NavPhase::BOOT_CHECK:
    case NavPhase::IDLE:
    default:
      return {"STOP", 0};
  }
}
}  // namespace

namespace {

struct StopBeforeSwitchCtx {
  bool armed = false;
  MotionCmd pending = {};
};

static StopBeforeSwitchCtx s_stop_before_switch;

static inline const char* decision_or_stop(const MotionCmd& cmd) {
  return (cmd.decision && cmd.decision[0]) ? cmd.decision : "STOP";
}

static inline bool is_stop_like(const char* decision) {
  if (!decision || !decision[0]) return true;
  return (strcmp(decision, "STOP") == 0) || (strcmp(decision, "IDLE") == 0);
}

static MotionCmd enforce_stop_before_switch(const MotionCmd& desired) {
  const char* desired_decision = decision_or_stop(desired);

  if (s_stop_before_switch.armed) {
    if (is_stop_like(desired_decision)) {
      s_stop_before_switch.armed = false;
      s_stop_before_switch.pending = {};
      return desired;
    }

    const char* pending_decision = decision_or_stop(s_stop_before_switch.pending);
    if (strcmp(desired_decision, pending_decision) != 0 || desired.percent != s_stop_before_switch.pending.percent) {
      s_stop_before_switch.pending = desired;
      return {"STOP", 0};
    }

    s_stop_before_switch.armed = false;
    return s_stop_before_switch.pending;
  }

  if (is_stop_like(desired_decision)) {
    return desired;
  }

  // Insert a STOP tick whenever we change motion mode (e.g., FORWARD -> ROTATE, STEP -> FORWARD).
  if (g_nav.decision.length() && g_nav.decision != desired_decision) {
    s_stop_before_switch.pending = desired;
    s_stop_before_switch.armed = true;
    return {"STOP", 0};
  }

  return desired;
}

}  // namespace

void handle_nav_command(const char* action) {
  const NavCommand cmd = parse_nav_command(action);
  if (cmd == NavCommand::None) return;

  // AsyncWebServer handlers may run on a different core/task than the Arduino loop.
  // Defer mutations of global nav/mission state to nav_tick() to avoid cross-core
  // data races (notably String mutations) that can lead to crashes/reboots.
  portENTER_CRITICAL(&s_nav_cmd_mux);
  s_pending_cmd = cmd;
  portEXIT_CRITICAL(&s_nav_cmd_mux);
}

void nav_tick(const AppState& sensors, AppState& writable) {
  const uint32_t now = millis();

  bool goto_h_cmd = false;
  switch (take_pending_command()) {
    case NavCommand::StartSystem:
      g_system_started = true;
      break;
    case NavCommand::Stop:
      g_system_started = false;
      g_nav.target_key = "";
      g_nav.target_label = "";
      mission_reset(NavPhase::BOOT_CHECK, now);
      break;
    case NavCommand::Hold:
      g_nav.target_key = "";
      g_nav.target_label = "";
      mission_reset(NavPhase::IDLE, now);
      break;
    case NavCommand::GotoPost:
      g_nav.target_key = "goto_post";
      g_nav.target_label = "";
      break;
    case NavCommand::GotoH:
      g_nav.target_key = "goto_h";
      g_nav.target_label = "";
      goto_h_cmd = true;
      break;
    case NavCommand::None:
    default:
      break;
  }

  g_nav.updated_ms = now;
  g_nav.system_started = g_system_started;
  g_nav.active = g_system_started;
  g_nav.heading_ready = sensors.imu.valid && isfinite(sensors.imu.yaw);
  g_nav.gps_ready = sensors.gps.valid;

  // Clear navigation outputs (kept for compatibility with telemetry/UI).
  g_nav.dest_lat = NAN;
  g_nav.dest_lng = NAN;
  g_nav.route_seq = g_route_seq;
  g_nav.distance_to_dest_m = NAN;
  g_nav.bearing_to_dest_deg = NAN;
  g_nav.segment_distance_m = NAN;
  g_nav.segment_remaining_m = NAN;
  g_nav.segment_eta_s = NAN;
  g_nav.segment_speed_mps = NAN;
  g_nav.segment_index = 0;
  g_nav.segment_total = 0;
  g_nav.path_len = 0;
  g_nav.current_path_idx = 0;
  memset(g_nav.path_ids, 0, sizeof(g_nav.path_ids));

  // GPS smoothing (2D Kalman): publish smoothed position into nav.ref_*.
  if (sensors.gps.valid && isfinite(sensors.gps.lat) && isfinite(sensors.gps.lng)) {
    double f_lat = NAN;
    double f_lng = NAN;
    g_gps_kf.update(sensors.gps.lat, sensors.gps.lng, &f_lat, &f_lng);
    g_nav.ref_lat = f_lat;
    g_nav.ref_lng = f_lng;
    g_nav.ref_valid = g_gps_kf.valid();
  } else {
    g_nav.ref_valid = g_gps_kf.valid();
  }
  g_nav.ref_active = sensors.gps.valid && g_nav.ref_valid;
  if (!g_nav.ref_valid) {
    g_nav.ref_lat = NAN;
    g_nav.ref_lng = NAN;
  }

  // BOOT_CHECK -> IDLE once the actuator checkup routine completed.
  if (!controller_state_active()) {
    if (g_mission.phase != NavPhase::BOOT_CHECK) {
      mission_transition(NavPhase::BOOT_CHECK, now);
    }
  } else if (g_mission.phase == NavPhase::BOOT_CHECK) {
    mission_transition(NavPhase::IDLE, now);
  }

  if (!g_system_started) {
    // System not started: remain idle, but keep showing readiness and sensor info.
    if (controller_state_active() && g_mission.phase != NavPhase::IDLE) {
      mission_transition(NavPhase::IDLE, now);
    }
  }

  if (g_system_started && controller_state_active()) {
    if (g_nav.target_key == "goto_h") {
      // Start (or explicitly restart) the mission only when requested; don't auto-restart
      // immediately after reaching DONE.
      if (g_mission.phase == NavPhase::IDLE || (goto_h_cmd && g_mission.phase == NavPhase::GOTO_H_DONE)) {
        mission_transition(NavPhase::GOTO_H_INIT, now);
        g_route_seq = (uint8_t)(g_route_seq + 1);
        g_nav.route_seq = g_route_seq;
      }
    } else {
      // Any non-go-to-H command (or none) puts the mission back to IDLE.
      if (g_mission.phase != NavPhase::IDLE && g_mission.phase != NavPhase::BOOT_CHECK) {
        mission_transition(NavPhase::IDLE, now);
      }
    }
  }

  g_nav.ready_for_goto = g_system_started && controller_state_active() && (g_mission.phase == NavPhase::IDLE);

  MotionCmd planned = {"STOP", 0};
  if (g_mission.phase == NavPhase::GOTO_H_INIT ||
      g_mission.phase == NavPhase::GOTO_H_LEG1_DRIVE ||
      g_mission.phase == NavPhase::GOTO_H_TURN1 ||
      g_mission.phase == NavPhase::GOTO_H_ALIGN1 ||
      g_mission.phase == NavPhase::GOTO_H_LEG2_DRIVE ||
      g_mission.phase == NavPhase::GOTO_H_TURN2 ||
      g_mission.phase == NavPhase::GOTO_H_WAIT_WALL2 ||
      g_mission.phase == NavPhase::GOTO_H_FIND_WALL2 ||
      g_mission.phase == NavPhase::GOTO_H_ALIGN2 ||
      g_mission.phase == NavPhase::GOTO_H_LEG3_DRIVE ||
      g_mission.phase == NavPhase::GOTO_H_LEG3_OPEN_DRIVE ||
      g_mission.phase == NavPhase::GOTO_H_TURN3 ||
      g_mission.phase == NavPhase::GOTO_H_ALIGN3 ||
      g_mission.phase == NavPhase::GOTO_H_LEG4_DRIVE ||
      g_mission.phase == NavPhase::GOTO_H_TURN4 ||
      g_mission.phase == NavPhase::GOTO_H_ALIGN4 ||
      g_mission.phase == NavPhase::GOTO_H_FINAL_APP ||
      g_mission.phase == NavPhase::GOTO_H_DONE) {
    planned = goto_h_tick(sensors, now);
  } else {
    planned = {"STOP", 0};
  }

  bool overridden = false;
  const MotionCmd desired = obstacle_tick(sensors, now, planned, &overridden);
  MotionCmd cmd = enforce_stop_before_switch(desired);
  if (!cmd.decision || (strcmp(cmd.decision, "STOP") == 0)) {
    cmd.percent = 0;
  } else if (cmd.percent == 0) {
    cmd.percent = 100;
  }

  apply_us_schedule_for_phase(g_mission.phase);
  if (g_obstacle.state == ObsState::AVOID_STEP_OUT || g_obstacle.state == ObsState::AVOID_FORWARD || g_obstacle.state == ObsState::AVOID_STEP_IN) {
    sensor_us_set_active(true, false, true, true, false);
  }

  g_nav.current_phase_enum = g_mission.phase;
  g_nav.phase = phase_name(g_mission.phase);
  g_nav.status = overridden ? "OBSTACLE" : ((g_mission.phase == NavPhase::IDLE) ? "idle" : "running");
  g_nav.decision = cmd.decision;
  g_nav.motion_percent = cmd.percent;
  g_nav.aim_yaw_deg = NAN;
  if (cmd.decision &&
      ((strcmp(cmd.decision, "ROTATE_LEFT") == 0) || (strcmp(cmd.decision, "ROTATE_RIGHT") == 0)) &&
      (isfinite(g_mission.turn_target_yaw) || (g_mission.drive_aim_set && isfinite(g_mission.drive_aim_yaw)))) {
    g_nav.aim_yaw_deg = isfinite(g_mission.turn_target_yaw) ? g_mission.turn_target_yaw : g_mission.drive_aim_yaw;
  }

  update_obstacle_snapshot(sensors, g_nav.obstacles);

  g_nav.status_line_1 = format_status_line(cmd, sensors);
  g_nav.status_line_2 = format_us_line(sensors.us);

  writable.nav = g_nav;
  writable.oled_label = g_nav.status_line_1 + "\n" + g_nav.status_line_2;
}

const NavigationState& nav_state() { return g_nav; }

}  // namespace nav
