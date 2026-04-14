#include "LidarAlign.h"
#include "LidarManager.h"
#include <math.h>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===================== Tunables =====================
static const bool INVERT_ROTATION_OUTPUT = false;
static const bool SWAP_LEFT_RIGHT_SECTORS = false;

// ======================= LD20 angle convention / mounting =======================
// Depending on firmware and mounting, LD20 may report angles with a different
// zero-direction and/or rotation direction.
//
// Choose a remap mode so that *robot-frame degrees* are:
//   0° = FRONT, 90° = LEFT, 180° = BACK, 270° = RIGHT  (CCW)
//
// If you see the symptom "FRONT <-> RIGHT" and "LEFT <-> BACK" swapped,
// then your raw scan is likely:
//   0° = RIGHT and the angle increases CLOCKWISE.
// In that case use mode 1.
//
// 0 = no remap (raw already matches robot-frame)
// 1 = robotDeg = wrapDeg(270 - rawDeg)  (raw: 0=RIGHT, CW)
static const uint8_t LIDAR_ORIENT_MODE = 1;

static const int VALID_MIN_MM = 200;

static const int WALL_MAX_DIST_MM = 4500;
static const int WALL_MIN_WIDTH_DEG = 30;
static const int WALL_FLATNESS_MM = 500;

static const int OBS_MAX_DIST_MM = 2500;
static const int OBS_MIN_WIDTH_DEG = 3;
static const int OBS_MAX_WIDTH_DEG = 25;
static const int OBS_CLUSTER_STEP_MM = 600;

static const int MINZONE_HALF_WIDTH_DEG = 10;

static const float ALIGN_TOL_DEG = 2.0f;
static const float ALIGN_KP = 2.0f;
static const float ALIGN_MIN_RATE = 8.0f;
static const float ALIGN_MAX_RATE = 50.0f;
static const float COARSE_RATE = 30.0f;
static const unsigned long COARSE_TIMEOUT_MS = 6000;

static const unsigned long FRONT_WALL_LATCH_MS = 700;

// ===== Free direction / drive tuning =====
// A direction is considered "free" if we see no return closer than FREE_MIN_MM.
// Note: the LD20 driver clamps distances to maxDistance=12000mm; real >12m often appears as 0 (no return).
static const int FREE_MIN_MM = 11800;    // treat >=11.8m (or 0) as "free"
static const int FREE_WIN_HALF_DEG = 3;  // +/- window for robustness (front check & scan)
static const float FREE_TURN_RATE_DEG_S = 25.0f;
static const float FREE_FORWARD_SPEED_MPS = 0.30f;
static const int FREE_STOP_DIST_MM = 1500;  // stop forward if something is closer than this

// ===================== Internals =====================
struct Segment {
  bool valid = false;
  int startDeg = 0;
  int endDeg = 0;
  int widthDeg = 0;
  int avgDist = 0;
  int minDist = 0;
  int centerDeg = 0;
};

struct SideResult {
  int minMm = 0;
  Segment wall;
  Segment obstacle;
  bool wallAhead = false;
  bool obstacleAhead = false;
};

struct AnalysisResult {
  SideResult front, left, right, back;
  uint32_t packets = 0;
  uint32_t crcErrors = 0;
  float crcRate = 0.0f;
};

struct SectorDef {
  const char* name;
  int startDeg;
  int endDeg;
};

enum class AlignState : uint8_t { Idle = 0,
                                  CoarseTurn = 1,
                                  FineAlign = 2 };
enum class WallSide : uint8_t { Left,
                                Right };
enum class TurnDir : uint8_t { Stop,
                               Left,
                               Right };

static inline int imax(int a, int b) {
  return (a > b) ? a : b;
}
static inline int imin(int a, int b) {
  return (a < b) ? a : b;
}
static inline float mmToM(int mm) {
  return (mm <= 0) ? 0.0f : ((float)mm / 1000.0f);
}

static inline int wrapDeg(int d) {
  while (d < 0) d += 360;
  while (d >= 360) d -= 360;
  return d;
}
static float wrapDeg180f(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// 8 Sector convention: 45 degrees each
// 0° = FRONT, 90° = LEFT, 180° = BACK, 270° = RIGHT
static const SectorDef SECTOR_FRONT = { "front", 338, 23 };               // 337-359, 0-22 (45°)
static const SectorDef SECTOR_FRONT_LEFT = { "front_left", 23, 67 };      // 23-67 (45°)
static const SectorDef SECTOR_LEFT = { "left", 68, 112 };                 // 68-112 (45°)
static const SectorDef SECTOR_BACK_LEFT = { "back_left", 113, 157 };      // 113-157 (45°)
static const SectorDef SECTOR_BACK = { "back", 158, 202 };                // 158-202 (45°)
static const SectorDef SECTOR_BACK_RIGHT = { "back_right", 203, 247 };    // 203-247 (45°)
static const SectorDef SECTOR_RIGHT = { "right", 248, 292 };              // 248-292 (45°)
static const SectorDef SECTOR_FRONT_RIGHT = { "front_right", 293, 336 };  // 293-336 (45°)

// For alignment, we need broader left/right sectors (spanning 3x45° = 135°)
static const SectorDef SECTOR_LEFT_WIDE = { "left_wide", 23, 157 };     // covers 135° (front-left + left + back-left)
static const SectorDef SECTOR_RIGHT_WIDE = { "right_wide", 203, 336 };  // covers 135° (back-right + right + front-right)

static const SectorDef& getSectorLeft() {
  return SWAP_LEFT_RIGHT_SECTORS ? SECTOR_RIGHT_WIDE : SECTOR_LEFT_WIDE;
}
static const SectorDef& getSectorRight() {
  return SWAP_LEFT_RIGHT_SECTORS ? SECTOR_LEFT_WIDE : SECTOR_RIGHT_WIDE;
}

// Shared LiDAR
static LidarManager gLidar;
static LidarManager::Config gCfg;

static AnalysisResult gRes;
static SideResult gSectorResults[8];
static int gLastDistMm[360] = { 0 };
static unsigned long gLastAnalyzeMs = 0;

// alignment FSM
static AlignState gAlignState = AlignState::Idle;
static WallSide gTargetSide = WallSide::Left;
static unsigned long gAlignStartMs = 0;
static float gAlignErrorDeg = NAN;


// Wall line angles (deg) computed continuously (NAN if not enough points / no wall)
static float gFrontWallLineAngleDeg = NAN;
static float gLeftWallLineAngleDeg = NAN;
static float gRightWallLineAngleDeg = NAN;
static float gBackWallLineAngleDeg = NAN;

// Facing error to ideal (deg)
static float gFrontWallErrDeg = NAN;  // ideal: ±90°
static float gLeftWallErrDeg = NAN;   // ideal: 0°
static float gRightWallErrDeg = NAN;  // ideal: 0°
static float gBackWallErrDeg = NAN;   // ideal: ±90°

static bool gDoneEvent = false;

// motion output
static LidarMotionCmd gOutCmd = { LIDAR_CMD_NONE, 0.0f, 0.0f };

// free scan (computed continuously)
static LidarFreeScanResult gFree12m;  // copy returned by lidarAlign_getFree12m()

// free drive FSM
typedef enum : uint8_t { FREE_IDLE = 0,
                         FREE_TURNING_LEFT,
                         FREE_DRIVING } FreeDriveState;
static FreeDriveState gFreeState = FREE_IDLE;
static bool gFreeDoneEvent = false;

// Free-drive message (debug/telemetry)
static const char* gFreeMsg = "";

// front wall latch
static bool gFrontWallLatched = false;
static unsigned long gFrontWallLastSeenMs = 0;

// ===================== motion helpers =====================
static void setCmdStop() {
  gOutCmd.type = LIDAR_CMD_STOP;
  gOutCmd.rateDeg_s = 0.0f;
  gOutCmd.speed_mps = 0.0f;
}
static void setCmdRotateLeft(float rate) {
  LidarCmdType t = LIDAR_CMD_ROTATE_LEFT;
  if (INVERT_ROTATION_OUTPUT) t = LIDAR_CMD_ROTATE_RIGHT;
  gOutCmd.type = t;
  gOutCmd.rateDeg_s = rate;
  gOutCmd.speed_mps = 0.0f;
}
static void setCmdRotateRight(float rate) {
  LidarCmdType t = LIDAR_CMD_ROTATE_RIGHT;
  if (INVERT_ROTATION_OUTPUT) t = LIDAR_CMD_ROTATE_LEFT;
  gOutCmd.type = t;
  gOutCmd.rateDeg_s = rate;
  gOutCmd.speed_mps = 0.0f;
}

static void setCmdForward(float speed_mps) {
  gOutCmd.type = LIDAR_CMD_FORWARD;
  gOutCmd.rateDeg_s = 0.0f;
  gOutCmd.speed_mps = speed_mps;
}

// ===================== detection helpers =====================
static int sectorMinInclusive(const int d[360], int startDeg, int endDeg) {
  int minv = 0;
  int a = startDeg;
  while (true) {
    int v = d[wrapDeg(a)];
    if (v >= VALID_MIN_MM) minv = (minv == 0) ? v : imin(minv, v);
    if (a == endDeg) break;
    a = wrapDeg(a + 1);
  }
  return minv;
}

static int sectorCenterDeg(const SectorDef& s) {
  if (s.endDeg >= s.startDeg) return wrapDeg(s.startDeg + (s.endDeg - s.startDeg) / 2);
  int span = (360 - s.startDeg) + (s.endDeg + 1);
  return wrapDeg(s.startDeg + span / 2);
}

static Segment findBestWall(const int d[360], int startDeg, int endDeg) {
  Segment best;
  auto isValid = [&](int v) {
    return (v >= VALID_MIN_MM && v <= WALL_MAX_DIST_MM);
  };

  int a = startDeg;
  int runStart = -1, runCount = 0;
  long runSum = 0;
  int runMin = 0;
  int prev = 0;
  bool first = true;

  while (true) {
    int v = d[wrapDeg(a)];
    bool ok = isValid(v);
    bool contiguousOk = ok && (first || (prev > 0 && abs(v - prev) <= WALL_FLATNESS_MM));

    if (contiguousOk) {
      if (runCount == 0) {
        runStart = a;
        runSum = 0;
        runMin = v;
      }
      runCount++;
      runSum += v;
      runMin = imin(runMin, v);
    } else {
      if (runCount >= WALL_MIN_WIDTH_DEG) {
        Segment s;
        s.valid = true;
        s.startDeg = wrapDeg(runStart);
        s.endDeg = wrapDeg(a - 1);
        s.widthDeg = runCount;
        s.avgDist = (int)(runSum / (long)imax(1, runCount));
        s.minDist = runMin;
        s.centerDeg = wrapDeg(runStart + (runCount / 2));

        long score = (long)s.widthDeg * 100000L / (long)imax(1, s.avgDist);
        long bestScore = best.valid ? ((long)best.widthDeg * 100000L / (long)imax(1, best.avgDist)) : -1;
        if (!best.valid || score > bestScore) best = s;
      }
      runCount = 0;
      first = true;
      prev = 0;
    }

    if (a == endDeg) break;
    prev = ok ? v : 0;
    first = false;
    a = wrapDeg(a + 1);
  }

  if (runCount >= WALL_MIN_WIDTH_DEG) {
    Segment s;
    s.valid = true;
    s.startDeg = wrapDeg(runStart);
    s.endDeg = wrapDeg(endDeg);
    s.widthDeg = runCount;
    s.avgDist = (int)(runSum / (long)imax(1, runCount));
    s.minDist = runMin;
    s.centerDeg = wrapDeg(runStart + (runCount / 2));

    long score = (long)s.widthDeg * 100000L / (long)imax(1, s.avgDist);
    long bestScore = best.valid ? ((long)best.widthDeg * 100000L / (long)imax(1, best.avgDist)) : -1;
    if (!best.valid || score > bestScore) best = s;
  }

  return best;
}

static Segment findBestWallRaw(const int d[360], int startDeg, int endDeg) {
  Segment best;
  auto isValid = [&](int v) {
    return (v >= VALID_MIN_MM && v <= WALL_MAX_DIST_MM);
  };

  int a = startDeg;
  int runStart = -1, runCount = 0;
  long runSum = 0;
  int runMin = 0;
  int prev = 0;
  bool first = true;

  while (true) {
    int v = d[wrapDeg(a)];
    bool ok = isValid(v);
    bool contiguousOk = ok && (first || (prev > 0 && abs(v - prev) <= WALL_FLATNESS_MM));

    if (contiguousOk) {
      if (runCount == 0) {
        runStart = a;
        runSum = 0;
        runMin = v;
      }
      runCount++;
      runSum += v;
      runMin = imin(runMin, v);
    } else {
      if (runCount > 0) {  // Track ANY run, not just >=30°
        Segment s;
        s.valid = false;  // Will set after width check
        s.startDeg = wrapDeg(runStart);
        s.endDeg = wrapDeg(a - 1);
        s.widthDeg = runCount;
        s.avgDist = (int)(runSum / (long)imax(1, runCount));
        s.minDist = runMin;
        s.centerDeg = wrapDeg(runStart + (runCount / 2));

        long score = (long)s.widthDeg * 100000L / (long)imax(1, s.avgDist);
        long bestScore = best.widthDeg > 0 ? ((long)best.widthDeg * 100000L / (long)imax(1, best.avgDist)) : -1;
        if (best.widthDeg == 0 || score > bestScore) best = s;
      }
      runCount = 0;
      first = true;
      prev = 0;
    }

    if (a == endDeg) break;
    prev = ok ? v : 0;
    first = false;
    a = wrapDeg(a + 1);
  }

  if (runCount > 0) {
    Segment s;
    s.valid = false;
    s.startDeg = wrapDeg(runStart);
    s.endDeg = wrapDeg(endDeg);
    s.widthDeg = runCount;
    s.avgDist = (int)(runSum / (long)imax(1, runCount));
    s.minDist = runMin;
    s.centerDeg = wrapDeg(runStart + (runCount / 2));

    long score = (long)s.widthDeg * 100000L / (long)imax(1, s.avgDist);
    long bestScore = best.widthDeg > 0 ? ((long)best.widthDeg * 100000L / (long)imax(1, best.avgDist)) : -1;
    if (best.widthDeg == 0 || score > bestScore) best = s;
  }

  return best;
}

static Segment findBestObs(const int d[360], int startDeg, int endDeg) {
  Segment best;
  auto isValid = [&](int v) {
    return (v >= VALID_MIN_MM && v <= OBS_MAX_DIST_MM);
  };

  int a = startDeg;
  int runStart = -1, runCount = 0;
  long runSum = 0;
  int runMin = 0;
  int prev = 0;
  bool first = true;

  while (true) {
    int v = d[wrapDeg(a)];
    bool ok = isValid(v);
    bool clustered = ok && (first || (prev > 0 && abs(v - prev) <= OBS_CLUSTER_STEP_MM));

    if (clustered) {
      if (runCount == 0) {
        runStart = a;
        runSum = 0;
        runMin = v;
      }
      runCount++;
      runSum += v;
      runMin = imin(runMin, v);
    } else {
      if (runCount >= OBS_MIN_WIDTH_DEG && runCount <= OBS_MAX_WIDTH_DEG) {
        Segment s;
        s.valid = true;
        s.startDeg = wrapDeg(runStart);
        s.endDeg = wrapDeg(a - 1);
        s.widthDeg = runCount;
        s.avgDist = (int)(runSum / (long)imax(1, runCount));
        s.minDist = runMin;
        s.centerDeg = wrapDeg(runStart + (runCount / 2));

        long score = (long)(100000L / (long)imax(1, s.minDist)) + (long)s.widthDeg * 50L;
        long bestScore = best.valid ? ((long)(100000L / (long)imax(1, best.minDist)) + (long)best.widthDeg * 50L) : -1;
        if (!best.valid || score > bestScore) best = s;
      }
      runCount = 0;
      first = true;
      prev = 0;
    }

    if (a == endDeg) break;
    prev = ok ? v : 0;
    first = false;
    a = wrapDeg(a + 1);
  }

  if (runCount >= OBS_MIN_WIDTH_DEG && runCount <= OBS_MAX_WIDTH_DEG) {
    Segment s;
    s.valid = true;
    s.startDeg = wrapDeg(runStart);
    s.endDeg = wrapDeg(endDeg);
    s.widthDeg = runCount;
    s.avgDist = (int)(runSum / (long)imax(1, runCount));
    s.minDist = runMin;
    s.centerDeg = wrapDeg(runStart + (runCount / 2));

    long score = (long)(100000L / (long)imax(1, s.minDist)) + (long)s.widthDeg * 50L;
    long bestScore = best.valid ? ((long)(100000L / (long)imax(1, best.minDist)) + (long)best.widthDeg * 50L) : -1;
    if (!best.valid || score > bestScore) best = s;
  }

  return best;
}

static void analyzeSector(const int d[360], const SectorDef& s, SideResult& out) {
  int center = sectorCenterDeg(s);
  int minStart = wrapDeg(center - MINZONE_HALF_WIDTH_DEG);
  int minEnd = wrapDeg(center + MINZONE_HALF_WIDTH_DEG);

  if (minEnd < minStart) {
    int a = sectorMinInclusive(d, minStart, 359);
    int b = sectorMinInclusive(d, 0, minEnd);
    out.minMm = (a == 0) ? b : ((b == 0) ? a : imin(a, b));
  } else {
    out.minMm = sectorMinInclusive(d, minStart, minEnd);
  }

  if (s.endDeg < s.startDeg) {
    Segment wA = findBestWall(d, s.startDeg, 359);
    Segment wB = findBestWall(d, 0, s.endDeg);
    long sA = wA.valid ? ((long)wA.widthDeg * 100000L / (long)imax(1, wA.avgDist)) : -1;
    long sB = wB.valid ? ((long)wB.widthDeg * 100000L / (long)imax(1, wB.avgDist)) : -1;
    out.wall = (!wA.valid) ? wB : (!wB.valid ? wA : (sA >= sB ? wA : wB));

    Segment oA = findBestObs(d, s.startDeg, 359);
    Segment oB = findBestObs(d, 0, s.endDeg);
    long oSA = oA.valid ? ((long)(100000L / (long)imax(1, oA.minDist)) + (long)oA.widthDeg * 50L) : -1;
    long oSB = oB.valid ? ((long)(100000L / (long)imax(1, oB.minDist)) + (long)oB.widthDeg * 50L) : -1;
    out.obstacle = (!oA.valid) ? oB : (!oB.valid ? oA : (oSA >= oSB ? oA : oB));
  } else {
    out.wall = findBestWall(d, s.startDeg, s.endDeg);
    out.obstacle = findBestObs(d, s.startDeg, s.endDeg);
  }

  out.wallAhead = out.wall.valid;
  out.obstacleAhead = out.obstacle.valid;
}

static void analyzeSectorFrontWrap(const int d[360], const SectorDef& s, SideResult& out) {
  // Special handling for front sector that wraps around 0°
  
  int center = sectorCenterDeg(s);
  int minStart = wrapDeg(center - MINZONE_HALF_WIDTH_DEG);
  int minEnd = wrapDeg(center + MINZONE_HALF_WIDTH_DEG);

  if (minEnd < minStart) {
    int a = sectorMinInclusive(d, minStart, 359);
    int b = sectorMinInclusive(d, 0, minEnd);
    out.minMm = (a == 0) ? b : ((b == 0) ? a : imin(a, b));
  } else {
    out.minMm = sectorMinInclusive(d, minStart, minEnd);
  }

  // Find wall segments in both halves (they might be <30° individually)
  Segment wA = findBestWallRaw(d, s.startDeg, 359);
  Segment wB = findBestWallRaw(d, 0, s.endDeg);
  
  // Try to merge even if individually invalid (under 30°)
  bool wallMerged = false;
  
  // Check if segments exist (have width >0) and connect at boundary
  if (wA.widthDeg > 0 && wB.widthDeg > 0) {
    int dist359 = d[359];
    int dist0 = d[0];
    if (dist359 >= VALID_MIN_MM && dist0 >= VALID_MIN_MM && abs(dist359 - dist0) <= WALL_FLATNESS_MM) {
      // Merge them
      Segment merged;
      merged.valid = false;  // will check width after merge
      merged.startDeg = wA.startDeg;
      merged.endDeg = wB.endDeg;
      merged.widthDeg = wA.widthDeg + wB.widthDeg;
      merged.avgDist = (wA.avgDist * wA.widthDeg + wB.avgDist * wB.widthDeg) / merged.widthDeg;
      merged.minDist = imin(wA.minDist, wB.minDist);
      merged.centerDeg = wrapDeg(wA.startDeg + (merged.widthDeg / 2));
      
      // NOW check if merged width meets threshold
      if (merged.widthDeg >= WALL_MIN_WIDTH_DEG) {
        merged.valid = true;
        out.wall = merged;
        wallMerged = true;
      }
    }
  }
  
  if (!wallMerged) {
    // Pick better segment (might both be invalid)
    long sA = wA.valid ? ((long)wA.widthDeg * 100000L / (long)imax(1, wA.avgDist)) : -1;
    long sB = wB.valid ? ((long)wB.widthDeg * 100000L / (long)imax(1, wB.avgDist)) : -1;
    out.wall = (!wA.valid) ? wB : (!wB.valid ? wA : (sA >= sB ? wA : wB));
  }

  // Same for obstacles
  Segment oA = findBestObs(d, s.startDeg, 359);
  Segment oB = findBestObs(d, 0, s.endDeg);
  
  bool obsMerged = false;
  if (oA.widthDeg > 0 && oB.widthDeg > 0) {
    int dist359 = d[359];
    int dist0 = d[0];
    if (dist359 >= VALID_MIN_MM && dist0 >= VALID_MIN_MM && abs(dist359 - dist0) <= OBS_CLUSTER_STEP_MM) {
      Segment merged;
      merged.valid = false;
      merged.startDeg = oA.startDeg;
      merged.endDeg = oB.endDeg;
      merged.widthDeg = oA.widthDeg + oB.widthDeg;
      merged.avgDist = (oA.avgDist * oA.widthDeg + oB.avgDist * oB.widthDeg) / merged.widthDeg;
      merged.minDist = imin(oA.minDist, oB.minDist);
      merged.centerDeg = wrapDeg(oA.startDeg + (merged.widthDeg / 2));
      
      if (merged.widthDeg >= OBS_MIN_WIDTH_DEG && merged.widthDeg <= OBS_MAX_WIDTH_DEG) {
        merged.valid = true;
        out.obstacle = merged;
        obsMerged = true;
      }
    }
  }
  
  if (!obsMerged) {
    long oSA = oA.valid ? ((long)(100000L / (long)imax(1, oA.minDist)) + (long)oA.widthDeg * 50L) : -1;
    long oSB = oB.valid ? ((long)(100000L / (long)imax(1, oB.minDist)) + (long)oB.widthDeg * 50L) : -1;
    out.obstacle = (!oA.valid) ? oB : (!oB.valid ? oA : (oSA >= oSB ? oA : oB));
  }

  out.wallAhead = out.wall.valid;
  out.obstacleAhead = out.obstacle.valid;
}

// ===================== free-space scan (12m) =====================
static bool dirFree12mWindow(const int d[360], int deg, bool* uncertainOut = nullptr) {
  int anyNonZero = 0;
  int minNonZero = 0;
  for (int k = -FREE_WIN_HALF_DEG; k <= FREE_WIN_HALF_DEG; k++) {
    int v = d[wrapDeg(deg + k)];
    if (v > 0) {
      anyNonZero++;
      minNonZero = (minNonZero == 0) ? v : imin(minNonZero, v);
    }
  }

  // If we got no returns at all, this is likely "beyond range" -> treat as free but flag as uncertain.
  if (anyNonZero == 0) {
    if (uncertainOut) *uncertainOut = true;
    return true;
  }

  if (uncertainOut) *uncertainOut = false;
  return (minNonZero >= FREE_MIN_MM);
}

static void computeFreeRanges12m(const int d[360], LidarFreeScanResult& out) {
  bool freeDeg[360];
  bool uncertainDeg[360];

  for (int deg = 0; deg < 360; deg++) {
    bool unc = false;
    bool ok = dirFree12mWindow(d, deg, &unc);
    freeDeg[deg] = ok;
    uncertainDeg[deg] = unc;
  }

  // collect raw ranges
  LidarFreeRange tmp[32];
  uint8_t tmpCount = 0;

  int i = 0;
  while (i < 360) {
    if (!freeDeg[i]) {
      i++;
      continue;
    }
    int start = i;
    bool anyUnc = uncertainDeg[i];
    int j = i;
    while (j + 1 < 360 && freeDeg[j + 1]) {
      j++;
      anyUnc = anyUnc || uncertainDeg[j];
    }
    int end = j;
    int width = end - start + 1;

    if (tmpCount < (uint8_t)(sizeof(tmp) / sizeof(tmp[0]))) {
      tmp[tmpCount].startDeg = start;
      tmp[tmpCount].endDeg = end;
      tmp[tmpCount].widthDeg = width;
      tmp[tmpCount].centerDeg = wrapDeg(start + (width / 2));
      tmp[tmpCount].uncertain = anyUnc;
      tmpCount++;
    }
    i = end + 1;
  }

  // merge wrap-around range if needed
  if (tmpCount >= 2 && freeDeg[0] && freeDeg[359]) {
    // find first (starts at 0) and last (ends at 359)
    int firstIdx = -1, lastIdx = -1;
    for (int k = 0; k < tmpCount; k++) {
      if (tmp[k].startDeg == 0) firstIdx = k;
      if (tmp[k].endDeg == 359) lastIdx = k;
    }
    if (firstIdx >= 0 && lastIdx >= 0 && firstIdx != lastIdx) {
      // merge into lastIdx: [start..359] U [0..end]
      tmp[lastIdx].endDeg = tmp[firstIdx].endDeg;
      tmp[lastIdx].widthDeg = (360 - tmp[lastIdx].startDeg) + (tmp[firstIdx].endDeg + 1);
      tmp[lastIdx].centerDeg = wrapDeg(tmp[lastIdx].startDeg + (tmp[lastIdx].widthDeg / 2));
      tmp[lastIdx].uncertain = tmp[lastIdx].uncertain || tmp[firstIdx].uncertain;

      // remove firstIdx by shifting
      for (int k = firstIdx; k + 1 < tmpCount; k++) tmp[k] = tmp[k + 1];
      tmpCount--;
    }
  }

  // keep the largest ranges (by width)
  out.count = 0;
  for (int k = 0; k < tmpCount; k++) {
    // insert into out.ranges (simple insertion sort by width)
    LidarFreeRange r = tmp[k];
    uint8_t pos = out.count;
    if (pos > LIDAR_MAX_FREE_RANGES) pos = LIDAR_MAX_FREE_RANGES;

    // find insertion point
    uint8_t ins = 0;
    while (ins < out.count && out.ranges[ins].widthDeg >= r.widthDeg) ins++;

    if (out.count < LIDAR_MAX_FREE_RANGES) {
      // shift right
      for (int m = (int)out.count; m > (int)ins; m--) out.ranges[m] = out.ranges[m - 1];
      out.ranges[ins] = r;
      out.count++;
    } else {
      // full: only insert if better than the smallest
      if (ins < LIDAR_MAX_FREE_RANGES) {
        for (int m = (int)LIDAR_MAX_FREE_RANGES - 1; m > (int)ins; m--) out.ranges[m] = out.ranges[m - 1];
        out.ranges[ins] = r;
        out.count = LIDAR_MAX_FREE_RANGES;
      }
    }
  }
}

// ===================== alignment math =====================
static float estimateSideWallAngleDeg(const int d[360], WallSide side) {
  const SectorDef& sec = (side == WallSide::Left) ? getSectorLeft() : getSectorRight();
  const int a = sec.startDeg, b = sec.endDeg;

  const int MAXP = 160;
  float xs[MAXP], ys[MAXP];
  int n = 0;

  for (int deg = a; deg <= b; deg++) {
    int r = d[wrapDeg(deg)];
    if (r < VALID_MIN_MM || r > WALL_MAX_DIST_MM) continue;

    float th = (float)deg * (float)M_PI / 180.0f;
    float x = (float)r * cosf(th);
    float y = (float)r * sinf(th);

    if (side == WallSide::Left && y < 50.0f) continue;
    if (side == WallSide::Right && y > -50.0f) continue;

    xs[n] = x;
    ys[n] = y;
    n++;
    if (n >= MAXP) break;
  }

  if (n < 15) return NAN;

  float mx = 0, my = 0;
  for (int i = 0; i < n; i++) {
    mx += xs[i];
    my += ys[i];
  }
  mx /= n;
  my /= n;

  float Sxx = 0, Syy = 0, Sxy = 0;
  for (int i = 0; i < n; i++) {
    float dx = xs[i] - mx, dy = ys[i] - my;
    Sxx += dx * dx;
    Syy += dy * dy;
    Sxy += dx * dy;
  }

  float angle = 0.5f * atan2f(2.0f * Sxy, (Sxx - Syy));
  float angleDeg = angle * 180.0f / (float)M_PI;

  float a0 = wrapDeg180f(angleDeg);
  float a180 = wrapDeg180f(angleDeg + 180.0f);
  return (fabsf(a0) < fabsf(a180)) ? a0 : a180;
}


enum class WallWhich : uint8_t { Front,
                                 Left,
                                 Right,
                                 Back };

static float estimateWallLineAngleDegInSector(const int d[360], const SectorDef& sec, WallWhich which) {
  const int MAXP = 180;
  float xs[MAXP], ys[MAXP];
  int n = 0;

  int deg = sec.startDeg;
  while (true) {
    int r = d[wrapDeg(deg)];
    if (r >= VALID_MIN_MM && r <= WALL_MAX_DIST_MM) {
      float th = (float)deg * (float)M_PI / 180.0f;
      float x = (float)r * cosf(th);
      float y = (float)r * sinf(th);

      // Gate points so we mainly fit the intended side
      bool keep = true;
      switch (which) {
        case WallWhich::Front: keep = (x > 50.0f); break;
        case WallWhich::Back: keep = (x < -50.0f); break;
        case WallWhich::Left: keep = (y > 50.0f); break;
        case WallWhich::Right: keep = (y < -50.0f); break;
      }

      if (keep) {
        xs[n] = x;
        ys[n] = y;
        n++;
        if (n >= MAXP) break;
      }
    }

    if (deg == sec.endDeg) break;
    deg = wrapDeg(deg + 1);
  }

  if (n < 15) return NAN;

  float mx = 0, my = 0;
  for (int i = 0; i < n; i++) {
    mx += xs[i];
    my += ys[i];
  }
  mx /= n;
  my /= n;

  float Sxx = 0, Syy = 0, Sxy = 0;
  for (int i = 0; i < n; i++) {
    float dx = xs[i] - mx;
    float dy = ys[i] - my;
    Sxx += dx * dx;
    Syy += dy * dy;
    Sxy += dx * dy;
  }

  float angle = 0.5f * atan2f(2.0f * Sxy, (Sxx - Syy));
  float angleDeg = angle * 180.0f / (float)M_PI;

  // choose representation closer to 0° (gives range [-90..90])
  float a0 = wrapDeg180f(angleDeg);
  float a180 = wrapDeg180f(angleDeg + 180.0f);
  return (fabsf(a0) < fabsf(a180)) ? a0 : a180;
}

static float wallFacingErrDeg(WallWhich which, float lineAngleDeg) {
  if (!isfinite(lineAngleDeg)) return NAN;
  if (which == WallWhich::Front || which == WallWhich::Back) {
    // Ideal is ±90° => error is 0 when |angle|=90
    return fabsf(90.0f - fabsf(lineAngleDeg));
  }
  // Ideal is 0°
  return fabsf(lineAngleDeg);
}

static bool frontWallAllowedNow(unsigned long nowMs) {
  if (!gFrontWallLatched) return false;
  return (nowMs - gFrontWallLastSeenMs) <= FRONT_WALL_LATCH_MS;
}

static void startAlign(WallSide side) {
  unsigned long now = millis();
  if (!frontWallAllowedNow(now)) {
    gAlignState = AlignState::Idle;
    gAlignErrorDeg = NAN;
    setCmdStop();
    return;
  }
  gTargetSide = side;
  gAlignStartMs = now;
  gAlignState = AlignState::CoarseTurn;
  gAlignErrorDeg = NAN;
  gDoneEvent = false;
}

static void stopAlign() {
  gAlignState = AlignState::Idle;
  gAlignErrorDeg = NAN;
  setCmdStop();
}

static void updateAlignFSM() {
  unsigned long now = millis();
  if (gAlignState == AlignState::Idle) return;

  if (!frontWallAllowedNow(now)) {
    stopAlign();
    return;
  }

  if (gAlignState == AlignState::CoarseTurn) {
    if (now - gAlignStartMs > COARSE_TIMEOUT_MS) {
      stopAlign();
      return;
    }

    bool sideWall = (gTargetSide == WallSide::Left) ? gRes.left.wallAhead : gRes.right.wallAhead;
    if (sideWall) {
      setCmdStop();
      gAlignState = AlignState::FineAlign;
      return;
    }

    // to move front wall into LEFT sector => rotate RIGHT
    if (gTargetSide == WallSide::Left) setCmdRotateRight(COARSE_RATE);
    else setCmdRotateLeft(COARSE_RATE);
    gAlignErrorDeg = NAN;
    return;
  }

  if (gAlignState == AlignState::FineAlign) {
    float err = estimateSideWallAngleDeg(gLastDistMm, gTargetSide);
    gAlignErrorDeg = err;

    if (!isfinite(err)) {
      stopAlign();
      return;
    }

    if (fabsf(err) <= ALIGN_TOL_DEG) {
      setCmdStop();
      gAlignState = AlignState::Idle;
      gDoneEvent = true;
      return;
    }

    float rate = fabsf(err) * ALIGN_KP;
    if (rate < ALIGN_MIN_RATE) rate = ALIGN_MIN_RATE;
    if (rate > ALIGN_MAX_RATE) rate = ALIGN_MAX_RATE;

    // positive error => rotate RIGHT
    if (err > 0) setCmdRotateRight(rate);
    else setCmdRotateLeft(rate);
  }
}

// ===================== free-drive (turn left to free, then forward) =====================
static void stopFreeDriveInternal(const char* msg, bool doneEvent) {
  setCmdStop();
  gFreeState = FREE_IDLE;
  gFreeDoneEvent = doneEvent;
  gFreeMsg = msg;
}

static void updateFreeDriveFSM() {
  if (gFreeState == FREE_IDLE) return;

  // Make modes mutually exclusive: free-drive overrides alignment.
  if (gAlignState != AlignState::Idle) {
    gAlignState = AlignState::Idle;
  }

  if (gFreeState == FREE_TURNING_LEFT) {
    bool uncertain = false;
    bool frontFree = dirFree12mWindow(gLastDistMm, 0, &uncertain);
    if (frontFree) {
      setCmdStop();
      gFreeState = FREE_DRIVING;
      gFreeMsg = uncertain ? "Front appears free (>12m, uncertain). Driving forward." : "Front is free (>12m). Driving forward.";
      return;
    }
    setCmdRotateLeft(FREE_TURN_RATE_DEG_S);
    gFreeMsg = "Turning LEFT to find a free direction (>12m in front)";
    return;
  }

  if (gFreeState == FREE_DRIVING) {
    // Stop condition: something appears within FREE_STOP_DIST_MM in front.
    const int f = gRes.front.minMm;
    if (f > 0 && f < FREE_STOP_DIST_MM) {
      stopFreeDriveInternal("STOP: obstacle/wall detected ahead", true);
      return;
    }
    setCmdForward(FREE_FORWARD_SPEED_MPS);
    gFreeMsg = "Driving forward (until obstacle appears)";
    return;
  }
}

// ===================== Public API =====================
void lidarAlign_init(int rxPin, int txPin) {
  gCfg.rxPin = rxPin;
  gCfg.txPin = txPin;
  gCfg.baudRate = 230400;
  gCfg.maxDistance = 12000;
  gCfg.ledPin = -1;

  gLidar.begin(Serial1, gCfg);

  gOutCmd.type = LIDAR_CMD_NONE;
  gOutCmd.rateDeg_s = 0.0f;
  gOutCmd.speed_mps = 0.0f;

  gAlignState = AlignState::Idle;
  gFrontWallLatched = false;
  gFrontWallLastSeenMs = 0;
  gDoneEvent = false;

  gFreeState = FREE_IDLE;
  gFreeDoneEvent = false;
  gFreeMsg = "";
  memset(&gFree12m, 0, sizeof(gFree12m));
}

void lidarAlign_update() {
  gLidar.update();

  unsigned long now = millis();
  if (now - gLastAnalyzeMs < 120) return;
  gLastAnalyzeMs = now;

  auto scan = gLidar.getScanData();
  if (!scan.dataReady) return;

  // --- Orientation remap (fix swapped sectors) ---
  // We want the *robot-frame* convention everywhere below:
  //   0° = FRONT, 90° = LEFT, 180° = BACK, 270° = RIGHT (counter-clockwise)
  // Some LD20 firmwares (and/or mounting orientations) yield different conventions.
  // Select the mapping at the top of this file via LIDAR_ORIENT_MODE.
  int mapped[360];
  for (int i = 0; i < 360; i++) mapped[i] = 0;
  switch (LIDAR_ORIENT_MODE) {
    default:
    case 0:  // no remap
      memcpy(mapped, scan.distanceMm, sizeof(mapped));
      break;
    case 1:  // raw: 0°=RIGHT, angle increases CLOCKWISE
      for (int oldIdx = 0; oldIdx < 360; oldIdx++) {
        int newIdx = wrapDeg(270 - oldIdx);
        mapped[newIdx] = scan.distanceMm[oldIdx];
      }
      break;
    case 2:  // rotate +90° (raw 0°=LEFT)
      for (int oldIdx = 0; oldIdx < 360; oldIdx++) {
        int newIdx = wrapDeg(oldIdx + 90);
        mapped[newIdx] = scan.distanceMm[oldIdx];
      }
      break;
    case 3:  // rotate -90° (raw 0°=RIGHT but CCW)
      for (int oldIdx = 0; oldIdx < 360; oldIdx++) {
        int newIdx = wrapDeg(oldIdx + 270);
        mapped[newIdx] = scan.distanceMm[oldIdx];
      }
      break;
    case 4:  // invert direction (mirror)
      for (int oldIdx = 0; oldIdx < 360; oldIdx++) {
        int newIdx = wrapDeg(360 - oldIdx);
        mapped[newIdx] = scan.distanceMm[oldIdx];
      }
      break;
  }

  // store for alignment math (line angle estimation + free-drive)
  memcpy(gLastDistMm, mapped, sizeof(gLastDistMm));

  AnalysisResult r;
  r.packets = scan.packetsReceived;
  r.crcErrors = scan.crcErrors;
  uint32_t total = r.packets + r.crcErrors;
  r.crcRate = (total > 0) ? (r.crcErrors * 100.0f) / (float)total : 0.0f;

  // Analyze all 8 sectors individually
  SideResult frontCenter, frontLeft, frontRight;
  SideResult leftCenter, backLeft;
  SideResult rightCenter, backRight;
  SideResult backCenter;

  analyzeSectorFrontWrap(mapped, SECTOR_FRONT, frontCenter);
  analyzeSector(mapped, SECTOR_FRONT_LEFT, frontLeft);
  analyzeSector(mapped, SECTOR_FRONT_RIGHT, frontRight);
  analyzeSector(mapped, SECTOR_LEFT, leftCenter);
  analyzeSector(mapped, SECTOR_BACK_LEFT, backLeft);
  analyzeSector(mapped, SECTOR_RIGHT, rightCenter);
  analyzeSector(mapped, SECTOR_BACK_RIGHT, backRight);
  analyzeSector(mapped, SECTOR_BACK, backCenter);

  gSectorResults[0] = frontCenter;
  gSectorResults[1] = frontLeft;
  gSectorResults[2] = leftCenter;
  gSectorResults[3] = backLeft;
  gSectorResults[4] = backCenter;
  gSectorResults[5] = backRight;
  gSectorResults[6] = rightCenter;
  gSectorResults[7] = frontRight;

  // Combine sectors for the 4 cardinal directions
  // FRONT = combine FRONT + FRONT_LEFT + FRONT_RIGHT
  r.front = frontCenter;
  r.front.minMm = imin(imin(frontCenter.minMm, frontLeft.minMm), frontRight.minMm);
  if (r.front.minMm == 0) r.front.minMm = imax(imax(frontCenter.minMm, frontLeft.minMm), frontRight.minMm);

  // Choose best wall from the 3 front sectors
  if (frontLeft.wallAhead && (!r.front.wallAhead || frontLeft.wall.widthDeg > r.front.wall.widthDeg)) {
    r.front.wall = frontLeft.wall;
  }
  if (frontRight.wallAhead && (!r.front.wallAhead || frontRight.wall.widthDeg > r.front.wall.widthDeg)) {
    r.front.wall = frontRight.wall;
  }
  r.front.wallAhead = r.front.wallAhead || frontLeft.wallAhead || frontRight.wallAhead;

  // Choose best obstacle from the 3 front sectors
  if (frontLeft.obstacleAhead && (!r.front.obstacleAhead || frontLeft.obstacle.minDist < r.front.obstacle.minDist)) {
    r.front.obstacle = frontLeft.obstacle;
  }
  if (frontRight.obstacleAhead && (!r.front.obstacleAhead || frontRight.obstacle.minDist < r.front.obstacle.minDist)) {
    r.front.obstacle = frontRight.obstacle;
  }
  r.front.obstacleAhead = r.front.obstacleAhead || frontLeft.obstacleAhead || frontRight.obstacleAhead;

  // LEFT = combine LEFT + BACK_LEFT + FRONT_LEFT
  r.left = leftCenter;
  r.left.minMm = imin(imin(leftCenter.minMm, backLeft.minMm), frontLeft.minMm);
  if (r.left.minMm == 0) r.left.minMm = imax(imax(leftCenter.minMm, backLeft.minMm), frontLeft.minMm);

  if (backLeft.wallAhead && (!r.left.wallAhead || backLeft.wall.widthDeg > r.left.wall.widthDeg)) {
    r.left.wall = backLeft.wall;
  }
  if (frontLeft.wallAhead && (!r.left.wallAhead || frontLeft.wall.widthDeg > r.left.wall.widthDeg)) {
    r.left.wall = frontLeft.wall;
  }
  r.left.wallAhead = r.left.wallAhead || backLeft.wallAhead || frontLeft.wallAhead;

  if (backLeft.obstacleAhead && (!r.left.obstacleAhead || backLeft.obstacle.minDist < r.left.obstacle.minDist)) {
    r.left.obstacle = backLeft.obstacle;
  }
  if (frontLeft.obstacleAhead && (!r.left.obstacleAhead || frontLeft.obstacle.minDist < r.left.obstacle.minDist)) {
    r.left.obstacle = frontLeft.obstacle;
  }
  r.left.obstacleAhead = r.left.obstacleAhead || backLeft.obstacleAhead || frontLeft.obstacleAhead;

  // RIGHT = combine RIGHT + BACK_RIGHT + FRONT_RIGHT
  r.right = rightCenter;
  r.right.minMm = imin(imin(rightCenter.minMm, backRight.minMm), frontRight.minMm);
  if (r.right.minMm == 0) r.right.minMm = imax(imax(rightCenter.minMm, backRight.minMm), frontRight.minMm);

  if (backRight.wallAhead && (!r.right.wallAhead || backRight.wall.widthDeg > r.right.wall.widthDeg)) {
    r.right.wall = backRight.wall;
  }
  if (frontRight.wallAhead && (!r.right.wallAhead || frontRight.wall.widthDeg > r.right.wall.widthDeg)) {
    r.right.wall = frontRight.wall;
  }
  r.right.wallAhead = r.right.wallAhead || backRight.wallAhead || frontRight.wallAhead;

  if (backRight.obstacleAhead && (!r.right.obstacleAhead || backRight.obstacle.minDist < r.right.obstacle.minDist)) {
    r.right.obstacle = backRight.obstacle;
  }
  if (frontRight.obstacleAhead && (!r.right.obstacleAhead || frontRight.obstacle.minDist < r.right.obstacle.minDist)) {
    r.right.obstacle = frontRight.obstacle;
  }
  r.right.obstacleAhead = r.right.obstacleAhead || backRight.obstacleAhead || frontRight.obstacleAhead;

  // BACK = combine BACK + BACK_LEFT + BACK_RIGHT
  r.back = backCenter;
  r.back.minMm = imin(imin(backCenter.minMm, backLeft.minMm), backRight.minMm);
  if (r.back.minMm == 0) r.back.minMm = imax(imax(backCenter.minMm, backLeft.minMm), backRight.minMm);

  if (backLeft.wallAhead && (!r.back.wallAhead || backLeft.wall.widthDeg > r.back.wall.widthDeg)) {
    r.back.wall = backLeft.wall;
  }
  if (backRight.wallAhead && (!r.back.wallAhead || backRight.wall.widthDeg > r.back.wall.widthDeg)) {
    r.back.wall = backRight.wall;
  }
  r.back.wallAhead = r.back.wallAhead || backLeft.wallAhead || backRight.wallAhead;

  if (backLeft.obstacleAhead && (!r.back.obstacleAhead || backLeft.obstacle.minDist < r.back.obstacle.minDist)) {
    r.back.obstacle = backLeft.obstacle;
  }
  if (backRight.obstacleAhead && (!r.back.obstacleAhead || backRight.obstacle.minDist < r.back.obstacle.minDist)) {
    r.back.obstacle = backRight.obstacle;
  }
  r.back.obstacleAhead = r.back.obstacleAhead || backLeft.obstacleAhead || backRight.obstacleAhead;

  gRes = r;

  // --- Wall line angles on all four sides (available before/during/after align) ---
  // Use the wider sectors for wall line estimation
  gFrontWallLineAngleDeg = gRes.front.wallAhead ? estimateWallLineAngleDegInSector(gLastDistMm, SECTOR_FRONT, WallWhich::Front) : NAN;
  gBackWallLineAngleDeg = gRes.back.wallAhead ? estimateWallLineAngleDegInSector(gLastDistMm, SECTOR_BACK, WallWhich::Back) : NAN;
  gLeftWallLineAngleDeg = gRes.left.wallAhead ? estimateWallLineAngleDegInSector(gLastDistMm, getSectorLeft(), WallWhich::Left) : NAN;
  gRightWallLineAngleDeg = gRes.right.wallAhead ? estimateWallLineAngleDegInSector(gLastDistMm, getSectorRight(), WallWhich::Right) : NAN;

  gFrontWallErrDeg = wallFacingErrDeg(WallWhich::Front, gFrontWallLineAngleDeg);
  gBackWallErrDeg = wallFacingErrDeg(WallWhich::Back, gBackWallLineAngleDeg);
  gLeftWallErrDeg = wallFacingErrDeg(WallWhich::Left, gLeftWallLineAngleDeg);
  gRightWallErrDeg = wallFacingErrDeg(WallWhich::Right, gRightWallLineAngleDeg);

  // compute free directions (12m) from latest scan (available continuously)
  computeFreeRanges12m(gLastDistMm, gFree12m);

  // latch front wall
  if (gRes.front.wallAhead) {
    gFrontWallLatched = true;
    gFrontWallLastSeenMs = now;
  } else {
    if (gFrontWallLatched && (now - gFrontWallLastSeenMs > FRONT_WALL_LATCH_MS)) {
      gFrontWallLatched = false;
    }
  }

  // default output command: NONE (unless a behaviour is active)
  if (gAlignState == AlignState::Idle && gFreeState == FREE_IDLE) {
    gOutCmd.type = LIDAR_CMD_NONE;
    gOutCmd.rateDeg_s = 0.0f;
    gOutCmd.speed_mps = 0.0f;
  }

  updateAlignFSM();
  updateFreeDriveFSM();
}

LidarDetectResult lidarAlign_getDetect() {
  LidarDetectResult d{};

  d.frontWall = gSectorResults[0].wallAhead;
  d.frontLeftWall = gSectorResults[1].wallAhead;
  d.leftWall = gSectorResults[2].wallAhead;
  d.backLeftWall = gSectorResults[3].wallAhead;
  d.backWall = gSectorResults[4].wallAhead;
  d.backRightWall = gSectorResults[5].wallAhead;
  d.rightWall = gSectorResults[6].wallAhead;
  d.frontRightWall = gSectorResults[7].wallAhead;

  d.frontObs = gSectorResults[0].obstacleAhead;
  d.frontLeftObs = gSectorResults[1].obstacleAhead;
  d.leftObs = gSectorResults[2].obstacleAhead;
  d.backLeftObs = gSectorResults[3].obstacleAhead;
  d.backObs = gSectorResults[4].obstacleAhead;
  d.backRightObs = gSectorResults[5].obstacleAhead;
  d.rightObs = gSectorResults[6].obstacleAhead;
  d.frontRightObs = gSectorResults[7].obstacleAhead;

  d.frontMin_m = mmToM(gSectorResults[0].minMm);
  d.frontLeftMin_m = mmToM(gSectorResults[1].minMm);
  d.leftMin_m = mmToM(gSectorResults[2].minMm);
  d.backLeftMin_m = mmToM(gSectorResults[3].minMm);
  d.backMin_m = mmToM(gSectorResults[4].minMm);
  d.backRightMin_m = mmToM(gSectorResults[5].minMm);
  d.rightMin_m = mmToM(gSectorResults[6].minMm);
  d.frontRightMin_m = mmToM(gSectorResults[7].minMm);

  d.frontWallLineAngleDeg = gFrontWallLineAngleDeg;
  d.leftWallLineAngleDeg = gLeftWallLineAngleDeg;
  d.rightWallLineAngleDeg = gRightWallLineAngleDeg;
  d.backWallLineAngleDeg = gBackWallLineAngleDeg;

  d.frontWallErrDeg = gFrontWallErrDeg;
  d.leftWallErrDeg = gLeftWallErrDeg;
  d.rightWallErrDeg = gRightWallErrDeg;
  d.backWallErrDeg = gBackWallErrDeg;

  d.packets = gRes.packets;
  d.crcErrors = gRes.crcErrors;
  d.crcRate = gRes.crcRate;
  d.alignAllowed = gFrontWallLatched;
  return d;
}

bool lidarAlign_canAlign() {
  return gFrontWallLatched;
}

uint8_t lidarAlign_state() {
  return (uint8_t)gAlignState;  // 0/1/2
}

float lidarAlign_errorDeg() {
  return isfinite(gAlignErrorDeg) ? gAlignErrorDeg : 999.0f;
}

void lidarAlign_startAlignLeft() {
  startAlign(WallSide::Left);
}
void lidarAlign_startAlignRight() {
  startAlign(WallSide::Right);
}

void lidarAlign_cancel() {
  // stop alignment + free-drive
  stopAlign();
  gDoneEvent = false;
  gFreeState = FREE_IDLE;
  gFreeDoneEvent = false;
  gFreeMsg = "";
  setCmdStop();
}

LidarMotionCmd lidarAlign_getMotionCmd() {
  return gOutCmd;
}

bool lidarAlign_takeDone() {
  bool v = gDoneEvent;
  gDoneEvent = false;
  return v;
}

// -------------------- free scan (12m) --------------------
LidarFreeScanResult lidarAlign_getFree12m() {
  return gFree12m;
}

// -------------------- free drive (12m) --------------------
void lidarAlign_startFreeDrive12m() {
  // Cancel alignment if active; the behaviours are mutually exclusive
  stopAlign();
  gDoneEvent = false;

  gFreeDoneEvent = false;
  gFreeState = FREE_TURNING_LEFT;
  gFreeMsg = "Turning left to find free direction";
}

bool lidarAlign_freeDriveActive() {
  return (gFreeState != FREE_IDLE);
}

bool lidarAlign_freeDriveTakeDone() {
  bool v = gFreeDoneEvent;
  gFreeDoneEvent = false;
  return v;
}

int lidarAlign_detectNarrowPole() {
  int count = 0;
  
  for (int deg = 255; deg <= 285; deg++) {
    int dist = gLastDistMm[wrapDeg(deg)];
    if (dist >= 250 && dist <= 600) {
      count++;
    }
  }
  
  return count;
}