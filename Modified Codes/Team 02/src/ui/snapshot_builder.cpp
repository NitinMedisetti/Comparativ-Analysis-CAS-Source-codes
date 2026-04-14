#include "snapshot_builder.h"

#include <math.h>

#include "cbor_codec.h"
#include <esp_timer.h>

// CBOR snapshot encoder for the dashboard.
// Keep this allocation-free and strictly bounded by `cap`.

static inline bool finite_or_null_float32(CborBuf& b, float v){
  if (isnan(v)) return cbor_null(b);
  return cbor_float32(b, v);
}

static inline bool finite_or_null_float64(CborBuf& b, double v){
  if (isnan(v)) return cbor_null(b);
  return cbor_float64(b, v);
}

size_t build_snapshot_cbor(const AppState& g, uint8_t* out, size_t cap){
  CborBuf buf; cbor_init(buf, out, cap);

  // Prebuild LiDAR ranges (360 x uint16 LE) into a temporary array
  uint8_t lidar_bytes[360*2];
  for (int i = 0; i < 360; ++i){
    uint16_t mm = 0;
    if (!isnan(g.lidarBins[i]) && g.lidarBins[i] > 0.0f){
      long v = lroundf(g.lidarBins[i]);
      if (v < 0) v = 0; if (v > 65535) v = 65535;
      mm = (uint16_t)v;
    }
    lidar_bytes[i*2+0] = (uint8_t)(mm & 0xFF);
    lidar_bytes[i*2+1] = (uint8_t)((mm >> 8) & 0xFF);
  }

  // Map with 7 keys
  if (!cbor_map(buf, 7)) return 0;

  // schema: "1.0.0"
  if (!cbor_tstr(buf, "schema")) return 0;
  if (!cbor_tstr(buf, "1.0.0")) return 0;

  // t_us: uint64
  if (!cbor_tstr(buf, "t_us")) return 0;
  if (!cbor_uint(buf, (uint64_t)esp_timer_get_time())) return 0;

  // imu
  if (!cbor_tstr(buf, "imu")) return 0;
  if (!cbor_map(buf, 17)) return 0;
  if (!cbor_tstr(buf, "yaw")) return 0; if (!finite_or_null_float32(buf, g.imu.yaw)) return 0;
  if (!cbor_tstr(buf, "pitch")) return 0; if (!finite_or_null_float32(buf, g.imu.pitch)) return 0;
  if (!cbor_tstr(buf, "roll")) return 0; if (!finite_or_null_float32(buf, g.imu.roll)) return 0;
  if (!cbor_tstr(buf, "lin_accel_x")) return 0; if (!finite_or_null_float32(buf, g.imu.linear_accel_x)) return 0;
  if (!cbor_tstr(buf, "lin_accel_y")) return 0; if (!finite_or_null_float32(buf, g.imu.linear_accel_y)) return 0;
  if (!cbor_tstr(buf, "lin_accel_z")) return 0; if (!finite_or_null_float32(buf, g.imu.linear_accel_z)) return 0;
  if (!cbor_tstr(buf, "accel_x")) return 0; if (!finite_or_null_float32(buf, g.imu.accel_x)) return 0;
  if (!cbor_tstr(buf, "accel_y")) return 0; if (!finite_or_null_float32(buf, g.imu.accel_y)) return 0;
  if (!cbor_tstr(buf, "accel_z")) return 0; if (!finite_or_null_float32(buf, g.imu.accel_z)) return 0;
  if (!cbor_tstr(buf, "gyro_x")) return 0; if (!finite_or_null_float32(buf, g.imu.gyro_x)) return 0;
  if (!cbor_tstr(buf, "gyro_y")) return 0; if (!finite_or_null_float32(buf, g.imu.gyro_y)) return 0;
  if (!cbor_tstr(buf, "gyro_z")) return 0; if (!finite_or_null_float32(buf, g.imu.gyro_z)) return 0;
  if (!cbor_tstr(buf, "mag_x")) return 0; if (!finite_or_null_float32(buf, g.imu.mag_x)) return 0;
  if (!cbor_tstr(buf, "mag_y")) return 0; if (!finite_or_null_float32(buf, g.imu.mag_y)) return 0;
  if (!cbor_tstr(buf, "mag_z")) return 0; if (!finite_or_null_float32(buf, g.imu.mag_z)) return 0;
  if (!cbor_tstr(buf, "temp_c")) return 0; if (!finite_or_null_float32(buf, g.imu.tempC)) return 0;
  if (!cbor_tstr(buf, "ok")) return 0; if (!(g.imu.valid ? cbor_true(buf) : cbor_false(buf))) return 0;

  // gps
  if (!cbor_tstr(buf, "gps")) return 0;
  if (!cbor_map(buf, 7)) return 0;
  if (!cbor_tstr(buf, "lat")) return 0; if (!finite_or_null_float64(buf, g.gps.lat)) return 0;
  if (!cbor_tstr(buf, "lon")) return 0; if (!finite_or_null_float64(buf, g.gps.lng)) return 0;
  if (!cbor_tstr(buf, "alt_m")) return 0; if (!finite_or_null_float32(buf, g.gps.alt_m)) return 0;
  if (!cbor_tstr(buf, "speed_mps")) return 0; {
    float mps = isnan(g.gps.speed_kmh) ? NAN : (g.gps.speed_kmh / 3.6f);
    if (!finite_or_null_float32(buf, mps)) return 0;
  }
  if (!cbor_tstr(buf, "sat")) return 0; if (!cbor_uint(buf, (uint64_t)g.gps.sats)) return 0;
  if (!cbor_tstr(buf, "fix")) return 0; if (!cbor_uint(buf, (uint64_t)(g.gps.valid ? 1 : 0))) return 0;
  if (!cbor_tstr(buf, "time_iso")) return 0; {
    // Combine date + time if available; else empty string
    if (g.gps.date.length() && g.gps.utc.length() && g.gps.utc != "null"){
      String iso = g.gps.date + "T" + g.gps.utc;
      if (!cbor_tstr(buf, iso.c_str())) return 0;
    } else {
      if (!cbor_tstr(buf, "")) return 0;
    }
  }

  // us
  if (!cbor_tstr(buf, "us")) return 0;
  if (!cbor_map(buf, 5)) return 0;
  if (!cbor_tstr(buf, "front")) return 0; if (!cbor_uint(buf, (uint64_t)max(0L, g.us.front_mm))) return 0;
  if (!cbor_tstr(buf, "back")) return 0; if (!cbor_uint(buf, (uint64_t)max(0L, g.us.back_mm))) return 0;
  if (!cbor_tstr(buf, "left")) return 0; if (!cbor_uint(buf, (uint64_t)max(0L, g.us.left_mm))) return 0;
  if (!cbor_tstr(buf, "right")) return 0; if (!cbor_uint(buf, (uint64_t)max(0L, g.us.right_mm))) return 0;
  if (!cbor_tstr(buf, "diag")) return 0; if (!cbor_uint(buf, (uint64_t)max(0L, g.us.d45_mm))) return 0;

  // lidar
  if (!cbor_tstr(buf, "lidar")) return 0;
  if (!cbor_map(buf, 2)) return 0;
  if (!cbor_tstr(buf, "ranges_mm_360")) return 0; if (!cbor_bstr(buf, lidar_bytes, sizeof(lidar_bytes))) return 0;
  if (!cbor_tstr(buf, "age_ms")) return 0; if (!cbor_uint(buf, (uint64_t)g.lidarHealth.last_ms)) return 0;

  // cfg
  if (!cbor_tstr(buf, "cfg")) return 0;
  if (!cbor_map(buf, 3)) return 0;
  if (!cbor_tstr(buf, "units")) return 0; if (!cbor_tstr(buf, g.units.length() ? g.units.c_str() : "mm")) return 0;
  if (!cbor_tstr(buf, "stabilize")) return 0; if (!(g.stabilize ? cbor_true(buf) : cbor_false(buf))) return 0;
  if (!cbor_tstr(buf, "cam_url")) return 0; if (!cbor_tstr(buf, g.cam_url.c_str())) return 0;

  // sys
  if (!cbor_tstr(buf, "sys")) return 0;
  if (!cbor_map(buf, 3)) return 0;
  if (!cbor_tstr(buf, "uptime_ms")) return 0; if (!cbor_uint(buf, (uint64_t)g.sys.uptime_ms)) return 0;
  if (!cbor_tstr(buf, "heap_free")) return 0; if (!cbor_uint(buf, (uint64_t)g.sys.free_heap)) return 0;
  if (!cbor_tstr(buf, "rssi")) return 0; if (!cbor_float32(buf, static_cast<float>(g.sys.wifi_rssi))) return 0;

  return buf.len;
}
