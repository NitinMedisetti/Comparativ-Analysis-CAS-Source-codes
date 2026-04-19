#include "sensor_lidar.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>

#include "core/controller_state.h"
#include "timebase.h"
#include "ui/ws_telemetry.h"

// LiDAR packet decoding + lightweight aggregation for UI/telemetry.
//
// The LD0x sensors stream packets with:
// - 12 points per packet
// - angles encoded in centi-degrees (0.01°)
// We reject packets that claim an unrealistically large angle span, because those
// are usually corrupted/"ghost" packets and would smear the 360° scan.
static constexpr uint8_t  kLidarPoints      = kLidarRawPoints;  // 12 points per packet
static constexpr size_t   kPacketSize       = 47;
static constexpr uint16_t kMaxRangeMm       = 8000;
static constexpr uint16_t kMinRangeMm       = 20;
static constexpr uint8_t  kMinIntensity     = 100;

// Sanity limit for a 12-point packet's start->end angle span.
static constexpr float    kMaxPacketSpanDeg = 15.0f;

// Aggregation window: accumulate raw points briefly, then collapse into 360 bins
// (one per degree) for UI + obstacle logic.
static constexpr size_t   kMaxWindowPoints  = 1000;       
static constexpr uint32_t kSweepTimeoutMs   = 250; 
static constexpr float    kSliceWidthDeg    = 1.0f;      
static constexpr size_t   kMaxSlices        = 360;       
static constexpr uint32_t kLidarHealthIntervalMs = 1000;
static constexpr uint32_t kLidarTelemetryIntervalMs = 500;
// Some setups prefer to accept packets that fail CRC if the framing is sane.
static constexpr bool     kProcessBadCrcPackets = true;

// Keep Serial1 parsing from monopolizing the main loop (WiFi/nav/controller).
// At 230400 baud, bytes arrive at ~23 bytes/ms, so a small budget still drains the stream quickly.
static constexpr uint32_t kPollBudgetUs = 2000;
static constexpr size_t kPollMaxBytes = 96;

// CRC lookup table for packet CRC8 validation.
static const uint8_t kLidarCrcTable[256] = {
  0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
  0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
  0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
  0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
  0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
  0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
  0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,
  0x03,0x4e,0x99,0xd4,0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,
  0xf7,0xba,0x6d,0x20,0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,
  0x5e,0x13,0xc4,0x89,0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,
  0xe8,0xa5,0x72,0x3f,0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,
  0x41,0x0c,0xdb,0x96,0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,
  0xc9,0x84,0x53,0x1e,0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,
  0x60,0x2d,0xfa,0xb7,0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,
  0xd6,0x9b,0x4c,0x01,0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,
  0x7f,0x32,0xe5,0xa8
};

static uint8_t LidarCalCRC8(const uint8_t* p, uint8_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; ++i) {
    crc = kLidarCrcTable[(crc ^ p[i]) & 0xFF];
  }
  return crc;
}

static float wrap_angle_360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

struct SliceSpec {
  float start_deg = NAN;
  float width_deg = NAN;
};

struct SliceAggregation {
  float mm = NAN;
  uint16_t used_points = 0;
};

// ===== Global State =====
static uint32_t s_lastFrameMs = 0;
static bool     s_debugDumpOnce = false;
static bool     s_debugStreaming = false;
static bool     s_rawStreaming = false;
static uint32_t s_last_lidar_health_ms = 0;

static uint32_t s_window_started_ms = 0;
static size_t   s_window_point_count = 0;
static LidarRawPoint s_window_points[kMaxWindowPoints];
static LidarRawFrame s_last_raw_frame;
static uint32_t s_lidar_ui_seq = 0;
static float    s_last_packet_end_deg = 0.0f;

static bool angle_in_slice(float deg, const SliceSpec& slice) {
  const float start = slice.start_deg;
  const float end = start + slice.width_deg;
  if (end <= 360.0f) {
    return deg >= start && deg < end;
  }
  return deg >= start || deg < (end - 360.0f);
}

struct ParsedPacket {
  LidarRawFrame frame;
  bool header_ok = false;
  bool len_ok = false;
  bool crc_ok = false;
  // Logical sanity checks (e.g., packet angle span).
  bool logic_ok = true;
};

static ParsedPacket decode_packet(const uint8_t* packet) {
  ParsedPacket result;
  
  const uint8_t ver_len = packet[1];
  const uint8_t point_count = ver_len & 0x1F;
  result.header_ok = packet[0] == 0x54;
  result.len_ok = point_count == kLidarPoints;
  result.crc_ok = (LidarCalCRC8(packet, kPacketSize - 1) == packet[kPacketSize - 1]);
  
  if (!result.header_ok || !result.len_ok) {
    return result;
  }

  // Decode packet header fields (speed + start/end angles).
  const uint16_t speed_raw  =  (uint16_t)packet[2] | ((uint16_t)packet[3] << 8);
  const uint16_t start_cdeg =  (uint16_t)packet[4] | ((uint16_t)packet[5] << 8);
  const uint16_t end_cdeg   = (uint16_t)packet[42] | ((uint16_t)packet[43] << 8);

  // Compute angular span between start and end angles (in centi-degrees).
  int32_t delta = (int32_t)end_cdeg - (int32_t)start_cdeg;
  if (delta < 0) delta += 36000;

  // Sanity check: a 12-point packet should only cover a small arc.
  // Convert 0.01° to degrees.
  const float span_deg = (float)delta / 100.0f;
  if (span_deg > kMaxPacketSpanDeg) {
    result.logic_ok = false;
    return result;
  }

  const float step_cdeg = delta / float(kLidarPoints - 1);

  LidarRawFrame frame = {};
  frame.ts_ms = millis();
  frame.rpm = int((float)speed_raw / 360.0f * 60.0f);
  frame.count = kLidarPoints;

  for (uint8_t i = 0; i < kLidarPoints; ++i) {
    const uint8_t  off       = 6 + i * 3;
    const uint16_t dist_mm   = (uint16_t)packet[off] | ((uint16_t)packet[off + 1] << 8);
    const uint8_t  intensity = packet[off + 2];

    const float angle_deg = (start_cdeg + step_cdeg * i) * 0.01f;
    frame.points[i].deg = wrap_angle_360(angle_deg);
    frame.points[i].mm = dist_mm;
    frame.points[i].intensity = intensity;
  }

  result.frame = frame;
  return result;
}

static bool filter_frame(const LidarRawFrame& raw, LidarRawFrame& filtered) {
  filtered = {};
  filtered.ts_ms = raw.ts_ms;
  filtered.rpm = raw.rpm;

  for (uint8_t i = 0; i < raw.count; ++i) {
    const auto& p = raw.points[i];
    if (p.intensity < kMinIntensity) continue;
    if (p.mm <= kMinRangeMm || p.mm >= kMaxRangeMm) continue;

    if (filtered.count < kLidarRawPoints) {
      filtered.points[filtered.count] = p;
      filtered.count += 1;
    }
  }
  return filtered.count > 0;
}

static void reset_window(uint32_t now_ms) {
  s_window_point_count = 0;
  s_window_started_ms = now_ms;
}

static void stash_window_points(const LidarRawFrame& frame) {
  for (uint8_t i = 0; i < frame.count; ++i) {
    if (s_window_point_count >= kMaxWindowPoints) break;
    s_window_points[s_window_point_count] = frame.points[i];
    s_window_point_count += 1;
  }
}

static size_t build_slice_plan(SliceSpec* slices, size_t max_count) {
  size_t count = 0;
  for (int deg = 0; deg < 360 && count < max_count; ++deg) {
    slices[count].start_deg = static_cast<float>(deg);
    slices[count].width_deg = kSliceWidthDeg;
    ++count;
  }
  return count;
}

// Simple outlier rejection: average points in a slice after dropping samples that
// deviate too far from the slice mean.
static SliceAggregation aggregate_slice(const SliceSpec& slice) {
  SliceAggregation result;
  if (slice.width_deg <= 0.0f) return result;

  double sum = 0.0;
  uint16_t count = 0;
  static float raw_values[10]; 
  uint8_t val_idx = 0;

  for (size_t i = 0; i < s_window_point_count; ++i) {
    const auto& p = s_window_points[i];
    if (!angle_in_slice(p.deg, slice)) continue;
    
    if (val_idx < 10) {
      raw_values[val_idx++] = p.mm;
      sum += p.mm;
      count++;
    }
  }

  if (count == 0) return result;

  float raw_mean = static_cast<float>(sum / count);
  double clean_sum = 0.0;
  uint16_t clean_count = 0;
  const float kMaxDeviationMm = 100.0f; 

  for (uint8_t i = 0; i < val_idx; ++i) {
    float diff = fabsf(raw_values[i] - raw_mean);
    if (diff <= kMaxDeviationMm) {
      clean_sum += raw_values[i];
      clean_count++;
    }
  }

  if (clean_count > 0) {
    result.mm = static_cast<float>(clean_sum / clean_count);
    result.used_points = clean_count;
  }
  return result;
}

static void populate_bins_from_slices(const SliceSpec* slices,
                                      const SliceAggregation* agg,
                                      size_t count,
                                      float* bins,
                                      uint32_t& used_samples) {
  for (int i = 0; i < 360; ++i) bins[i] = NAN;
  used_samples = 0;
  for (size_t i = 0; i < count; ++i) {
    if (isnan(agg[i].mm) || agg[i].used_points == 0) continue;
    used_samples += agg[i].used_points;
    const int deg = static_cast<int>(slices[i].start_deg);
    const int offset_deg = controller_state_expander_ready() ? 180 : 90;
    const int target_deg = (deg + offset_deg) % 360;
    bins[target_deg] = agg[i].mm;
  }
}

static void publish_lidar_ui(AppState& state, uint32_t now_ms) {
  if (s_window_point_count == 0) {
    reset_window(now_ms);
    return;
  }

  static SliceSpec slices[kMaxSlices];
  const size_t slice_count = build_slice_plan(slices, kMaxSlices);
  static SliceAggregation slice_results[kMaxSlices];
  
  for (size_t i = 0; i < slice_count; ++i) {
    slice_results[i] = aggregate_slice(slices[i]);
  }

  float bins[360];
  uint32_t used_samples = 0;
  populate_bins_from_slices(slices, slice_results, slice_count, bins, used_samples);

  for (int i = 0; i < 360; ++i) {
    state.lidarBins[i] = bins[i];
  }
  state.lidarBinsStamp.t_us = now_us();
  state.lidarBinsStamp.seq += 1;
  state.lidar_sweep_id += 1;

  static uint32_t last_send_ms = 0;
  const bool allow_send =
      (last_send_ms == 0) || ((uint32_t)(now_ms - last_send_ms) >= kLidarTelemetryIntervalMs);
  if (allow_send && ui::telemetry_ws_has_clients()) {
    last_send_ms = now_ms;

    JsonDocument doc;
    doc["type"] = "lidar_ui";
    doc["ts_ms"] = now_ms;
    doc["period_ms"] = (now_ms - s_window_started_ms);
    doc["seq"] = ++s_lidar_ui_seq;

    JsonArray bins_json = doc["bins"].to<JsonArray>();
    for (int i = 0; i < 360; ++i) {
      if (isnan(bins[i]) || bins[i] <= 0.0f) {
        bins_json.add(0);
      } else {
        float mm = bins[i];
        if (mm > static_cast<float>(kMaxRangeMm)) mm = static_cast<float>(kMaxRangeMm);
        bins_json.add(static_cast<uint16_t>(lroundf(mm)));
      }
    }
    ui::telemetry_ws_send(doc);
  }

  reset_window(now_ms);
}

static void handle_parsed_packet(const ParsedPacket& parsed, AppState& state, uint32_t& last_ok_ms) {
  s_last_raw_frame = parsed.frame;

  LidarRawFrame filtered;
  if (!filter_frame(parsed.frame, filtered)) {
    return;
  }

  if (s_window_started_ms == 0) s_window_started_ms = parsed.frame.ts_ms;
  
  stash_window_points(filtered);

  float current_start_deg = filtered.points[0].deg;
  bool sweep_completed = (s_last_packet_end_deg > 300.0f && current_start_deg < 60.0f);
  bool timeout_triggered = (parsed.frame.ts_ms - s_window_started_ms) > kSweepTimeoutMs;
  bool buffer_full = (s_window_point_count >= kMaxWindowPoints);

  if (sweep_completed || timeout_triggered || buffer_full) {
    publish_lidar_ui(state, parsed.frame.ts_ms);
  }

  if (filtered.count > 0) {
    s_last_packet_end_deg = filtered.points[filtered.count - 1].deg;
  }

  last_ok_ms = parsed.frame.ts_ms;
  state.lidarHealth.frame_ok = true;
  state.lidarHealth.rpm = filtered.rpm;
  state.lidar_packet_seq += 1;
  
  if (s_lastFrameMs != 0) {
    uint32_t dt = parsed.frame.ts_ms - s_lastFrameMs;
    if (dt > 0) state.lidarHealth.frame_hz = 1000.0f / (float)dt;
  }
  s_lastFrameMs = parsed.frame.ts_ms;
}

void sensor_lidar_begin(int rx_pin, int tx_pin){
  Serial1.begin(LIDAR_UART_BAUD, SERIAL_8N1, rx_pin, tx_pin);
  s_lastFrameMs = 0;
  reset_window(millis());
  s_last_packet_end_deg = 0.0f;
}

void sensor_lidar_poll(AppState& state){
  static uint8_t packet[kPacketSize];
  static size_t  index = 0;
  static uint32_t last_ok = 0;

  const uint32_t start_us = micros();
  size_t bytes_read = 0;
  while (Serial1.available() &&
         bytes_read < kPollMaxBytes &&
         (uint32_t)(micros() - start_us) < kPollBudgetUs) {
    uint8_t b = Serial1.read();
    bytes_read += 1;
    
    if (index == 0) {
      if (b != 0x54) continue;
      packet[index++] = b;
    } else {
      packet[index++] = b;
      if (index == kPacketSize) {
        ParsedPacket parsed = decode_packet(packet);
        // Accept only packets that pass framing + our sanity checks (CRC is optional).
        if (parsed.header_ok && parsed.len_ok && parsed.logic_ok && (parsed.crc_ok || kProcessBadCrcPackets)) {
          handle_parsed_packet(parsed, state, last_ok);
        }
        index = 0;
      }
    }
  }

  uint32_t now = millis();
  state.lidarHealth.last_ms = now - last_ok;
  if (state.lidarHealth.last_ms > 2000) {
    state.lidarHealth.frame_ok = false;
    state.lidarHealth.frame_hz = 0.0f;
    state.lidarHealth.rpm = 0;
  }

  if (now - s_last_lidar_health_ms >= kLidarHealthIntervalMs) {
    s_last_lidar_health_ms = now;
    if (ui::telemetry_ws_has_clients()) {
      JsonDocument healthDoc;
      healthDoc["type"] = "lidar_health";
      healthDoc["rpm"] = state.lidarHealth.rpm;
      healthDoc["ok"] = state.lidarHealth.frame_ok;
      ui::telemetry_ws_send(healthDoc);
    }
  }
}

void sensor_lidar_debug_request_once(){}
void sensor_lidar_debug_set_stream(bool enable){}
bool sensor_lidar_debug_streaming(){return false;}
void sensor_lidar_debug_set_raw_stream(bool enable){}
bool sensor_lidar_debug_raw_streaming(){return false;}
LidarRawFrame sensor_lidar_last_raw(){ return s_last_raw_frame; }
