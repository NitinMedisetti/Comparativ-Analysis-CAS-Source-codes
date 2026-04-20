#include "display_oled.h"
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// OLED dashboard renderer (SSD1306 128x64).
//
// Design goals:
// - Keep I2C traffic low (hash-based "only redraw when changed").
// - Keep the UI readable and stable (fixed text sizing/spacing).

namespace {

constexpr uint32_t kScreenTimeoutMs = 600000;
constexpr uint32_t kMinRefreshMs = 200;
constexpr uint32_t kSensorRefreshMs = 100;
constexpr uint32_t kCalHideDelayMs = 3000;
// Use a single text scale across the UI to avoid readability/layout issues when tuning sizes.
constexpr uint8_t kTextSize = 1;
constexpr int kLineHeightPx = 8 * kTextSize;
constexpr int kLineSpacingPx = 10;

constexpr int32_t kMissingI32 = 0x7fffffff;

uint32_t s_last_refresh_ms = 0;
uint32_t s_last_activity_ms = 0;
bool s_display_is_on = false;
String s_last_message;
bool s_last_dashboard_valid = false;
uint32_t s_last_dashboard_hash = 0;
uint32_t s_last_sensor_ms = 0;

// Cache yaw/US at a slower cadence so the OLED panel does not churn on every
// sensor tick (which can add noticeable latency to the main loop).
int32_t s_cached_yaw_deg = kMissingI32;
int32_t s_cached_us_front_mm = 0;
int32_t s_cached_us_right_mm = 0;
int32_t s_cached_us_left_mm = 0;

// Hide IMU calibration rows after the values stay at 3/3 for a short period.
uint32_t s_cal_complete_since_ms = 0;
bool s_cal_hidden = false;

// Column layout for the 128x64 SSD1306.
constexpr int kSep1X = 32;
constexpr int kSep2X = 88;
constexpr int kCalX = 0;
constexpr int kCenterIconX __attribute__((unused)) = 44;
constexpr int kCenterIconY __attribute__((unused)) = 2;
constexpr int kCenterInfoY __attribute__((unused)) = 42;
constexpr int kDistX = 90;
constexpr int kRow1Y = 8;
constexpr int kRow2Y = 16;
constexpr int kRow3Y = 24;
constexpr int kYawRateY = 32;

// 32x32 movement/status icons for the center column.
static const uint8_t PROGMEM icon_forward[] __attribute__((unused)) = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00,
  0x00, 0x01, 0xC0, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x07, 0xF0, 0x00, 0x00, 0x0F, 0xF8, 0x00,
  0x00, 0x1F, 0xFC, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x3F, 0xFE, 0x00, 0x00, 0x7F, 0xFF, 0x00,
  0x00, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0xC0, 0x03, 0xFF, 0xFF, 0xE0, 0x00, 0x07, 0xE0, 0x00,
  0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00,
  0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00,
  0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00,
  0x00, 0x1F, 0xF8, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t PROGMEM icon_backward[] __attribute__((unused)) = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x1F, 0xF8, 0x00,
  0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00,
  0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00,
  0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00,
  0x00, 0x07, 0xE0, 0x00, 0x03, 0xFF, 0xFF, 0xE0, 0x01, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0x80,
  0x00, 0x7F, 0xFF, 0x00, 0x00, 0x3F, 0xFE, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x1F, 0xFC, 0x00,
  0x00, 0x0F, 0xF8, 0x00, 0x00, 0x07, 0xF0, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x01, 0xC0, 0x00,
  0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t PROGMEM icon_step_left[] __attribute__((unused)) = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,
  0x00, 0x0E, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x70, 0xFE, 0x00, 0x00,
  0x71, 0xFE, 0x00, 0x00, 0x73, 0xFF, 0xFF, 0xFC, 0x77, 0xFF, 0xFF, 0xFC, 0x7F, 0xFF, 0xFF, 0xFC,
  0x7F, 0xFF, 0xFF, 0xFC, 0x7F, 0xFF, 0xFF, 0xFC, 0x77, 0xFF, 0xFF, 0xFC, 0x73, 0xFE, 0x00, 0x00,
  0x71, 0xFE, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00,
  0x00, 0x0E, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t PROGMEM icon_step_right[] __attribute__((unused)) = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00,
  0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x7F, 0x0E,
  0x00, 0x00, 0x7F, 0x8E, 0x3F, 0xFF, 0xFF, 0xCE, 0x3F, 0xFF, 0xFF, 0xEE, 0x3F, 0xFF, 0xFF, 0xFE,
  0x3F, 0xFF, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF, 0xEE, 0x00, 0x00, 0x7F, 0xCE,
  0x00, 0x00, 0x7F, 0x8E, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x78, 0x00,
  0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t PROGMEM icon_stop[] __attribute__((unused)) = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80,
  0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80,
  0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80,
  0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80,
  0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x80,
  0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t PROGMEM icon_idle[] __attribute__((unused)) = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x3F, 0xFE, 0x00,
  0x00, 0xF0, 0x07, 0x80, 0x00, 0xE0, 0x03, 0x80, 0x01, 0xC0, 0x01, 0xC0, 0x03, 0x80, 0x00, 0xE0,
  0x03, 0x00, 0x00, 0x60, 0x03, 0x00, 0x00, 0x60, 0x07, 0x00, 0x00, 0x70, 0x07, 0x01, 0x80, 0x70,
  0x07, 0x01, 0x80, 0x70, 0x07, 0x00, 0x00, 0x70, 0x07, 0x00, 0x00, 0x70, 0x03, 0x00, 0x00, 0x60,
  0x03, 0x00, 0x00, 0x60, 0x03, 0x80, 0x00, 0xE0, 0x01, 0xC0, 0x01, 0xC0, 0x00, 0xE0, 0x03, 0x80,
  0x00, 0xF0, 0x07, 0x80, 0x00, 0x3F, 0xFE, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x03, 0xE0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t PROGMEM icon_rotate_left[] __attribute__((unused)) = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x3F, 0xFE, 0x00,
  0x00, 0xF0, 0x07, 0x80, 0x00, 0xE0, 0x03, 0x80, 0x01, 0xC8, 0x01, 0xC0, 0x03, 0x98, 0x00, 0xE0,
  0x03, 0x38, 0x00, 0x60, 0x03, 0x78, 0x00, 0x60, 0x07, 0xF8, 0x00, 0x70, 0x07, 0xF8, 0x00, 0x70,
  0x07, 0xF8, 0x00, 0x70, 0x07, 0xF8, 0x00, 0x70, 0x07, 0xF8, 0x00, 0x70, 0x03, 0x78, 0x00, 0x60,
  0x03, 0x38, 0x00, 0x60, 0x03, 0x98, 0x00, 0xE0, 0x01, 0xC8, 0x01, 0xC0, 0x00, 0xE0, 0x03, 0x80,
  0x00, 0xF0, 0x07, 0x80, 0x00, 0x3F, 0xFE, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x03, 0xE0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t PROGMEM icon_rotate_right[] __attribute__((unused)) = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x3F, 0xFE, 0x00,
  0x00, 0xF0, 0x07, 0x80, 0x00, 0xE0, 0x03, 0x80, 0x01, 0xC0, 0x09, 0xC0, 0x03, 0x80, 0x0C, 0xE0,
  0x03, 0x00, 0x0E, 0x60, 0x03, 0x00, 0x0F, 0x60, 0x07, 0x00, 0x0F, 0xF0, 0x07, 0x00, 0x0F, 0xF0,
  0x07, 0x00, 0x0F, 0xF0, 0x07, 0x00, 0x0F, 0xF0, 0x07, 0x00, 0x0F, 0xF0, 0x03, 0x00, 0x0F, 0x60,
  0x03, 0x00, 0x0E, 0x60, 0x03, 0x80, 0x0C, 0xE0, 0x01, 0xC0, 0x09, 0xC0, 0x00, 0xE0, 0x03, 0x80,
  0x00, 0xF0, 0x07, 0x80, 0x00, 0x3F, 0xFE, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x03, 0xE0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// 8x8 UI icon used for IMU calibration status.
static const uint8_t PROGMEM icon_check[] = {0x00, 0x00, 0x02, 0x47, 0x6E, 0x3C, 0x18, 0x00};

static int text_width_px(const String& s, uint8_t text_size) {
  // Default GFX font: 5x7 with 1px spacing => 6px/char.
  return (int)s.length() * 6 * (int)text_size;
}

static String trim_copy(const String& s) {
  String out = s;
  out.trim();
  return out;
}

static void split_status_and_data(const String& message, String* out_status, String* out_data) {
  const int nl = message.indexOf('\n');
  if (nl < 0) {
    *out_status = trim_copy(message);
    *out_data = "";
    return;
  }
  *out_status = trim_copy(message.substring(0, nl));
  *out_data = trim_copy(message.substring(nl + 1));
}

static String fit_to_width(const String& text, int width_px, uint8_t text_size) {
  if (text_width_px(text, text_size) <= width_px) return text;
  if (width_px <= 0) return "";

  const int char_w = 6 * (int)text_size;
  const int max_chars = width_px / char_w;
  if (max_chars <= 0) return "";
  if ((int)text.length() <= max_chars) return text;

  // Use ASCII "..." to stay compatible with the default font.
  if (max_chars <= 3) {
    return text.substring(0, max_chars);
  }
  return text.substring(0, max_chars - 3) + "...";
}

static void draw_centered_text(Adafruit_SSD1306& display, const String& text, int y, uint8_t text_size) {
  const int W = display.width();
  const String clipped = fit_to_width(text, W, text_size);
  const int w_px = text_width_px(clipped, text_size);
  int x = (W - w_px) / 2;
  if (x < 0) x = 0;

  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setFont(NULL);
  display.setCursor(x, y);
  display.print(clipped);
}

static void draw_centered_text_in_column(Adafruit_SSD1306& display,
                                         const String& text,
                                         int col_x,
                                         int col_w,
                                         int y,
                                         uint8_t text_size) {
  const String clipped = fit_to_width(text, col_w, text_size);
  const int w_px = text_width_px(clipped, text_size);
  int x = col_x + (col_w - w_px) / 2;
  if (x < col_x) x = col_x;

  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setFont(NULL);
  display.setCursor(x, y);
  display.print(clipped);
}

static uint32_t fnv1a_add_u8(uint32_t h, uint8_t b) {
  h ^= (uint32_t)b;
  return h * 16777619u;
}

static uint32_t fnv1a_add_u32(uint32_t h, uint32_t v) {
  h = fnv1a_add_u8(h, (uint8_t)(v & 0xFFu));
  h = fnv1a_add_u8(h, (uint8_t)((v >> 8) & 0xFFu));
  h = fnv1a_add_u8(h, (uint8_t)((v >> 16) & 0xFFu));
  h = fnv1a_add_u8(h, (uint8_t)((v >> 24) & 0xFFu));
  return h;
}

static uint32_t fnv1a_add_string(uint32_t h, const String& s) {
  for (size_t i = 0; i < s.length(); ++i) {
    h = fnv1a_add_u8(h, (uint8_t)s[i]);
  }
  return h;
}

static String parse_action(const String& status_line) {
  String action = status_line;
  const int pipe = action.indexOf('|');
  if (pipe >= 0) action = action.substring(0, pipe);
  action.trim();
  action.toUpperCase();
  return action;
}

static const uint8_t* __attribute__((unused)) choose_center_icon(const String& action) {
  if (action == "FORWARD") return icon_forward;
  if (action == "BACKWARD") return icon_backward;
  if (action == "ROTATE_LEFT") return icon_rotate_left;
  if (action == "ROTATE_RIGHT") return icon_rotate_right;
  if (action == "STEP_LEFT") return icon_step_left;
  if (action == "STEP_RIGHT") return icon_step_right;
  if (action == "STOP") return icon_stop;
  if (action == "IDLE") return icon_idle;
  return icon_idle;
}

static String dashboard_action(const AppState& state) {
  if (state.nav.decision.length() > 0) {
    String action = state.nav.decision;
    action.trim();
    action.toUpperCase();
    return action;
  }

  String status_line;
  String data_line;
  split_status_and_data(state.oled_label, &status_line, &data_line);
  (void)data_line;
  return parse_action(status_line);
}

static String with_percent(const String& action, uint8_t percent) {
  return action + "(" + String((int)percent) + ")";
}

static void draw_cal_row(Adafruit_SSD1306& display, char label, uint8_t value, int y) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setFont(NULL);
  display.setCursor(kCalX + 2, y);
  display.print(label);
  display.print(':');

  const int val_x = kCalX + 18;
  if (value >= 3) {
    display.drawBitmap(val_x, y, icon_check, 8, 8, SSD1306_WHITE);
  } else {
    display.setCursor(val_x, y);
    display.print((int)value);
  }
}

static float wrap_deg(float deg) {
  if (!isfinite(deg)) return NAN;
  float w = fmodf(deg, 360.0f);
  if (w < 0.0f) w += 360.0f;
  return w;
}

static bool gps_trust_for_nav_distance(const AppState& state) {
  if (!state.gps.valid) return false;
  if (!isfinite(state.gps.hdop) || state.gps.hdop >= 2.0) return false;
  if (state.gps.sats < 7) return false;
  if (!state.nav.ref_valid) return false;
  if (!isfinite(state.nav.ref_lat) || !isfinite(state.nav.ref_lng)) return false;
  if (!isfinite(state.nav.dest_lat) || !isfinite(state.nav.dest_lng)) return false;
  return true;
}

static bool is_goto_h_drive_phase(NavPhase phase) {
  switch (phase) {
    case NavPhase::GOTO_H_LEG1_DRIVE:
    case NavPhase::GOTO_H_LEG2_DRIVE:
    case NavPhase::GOTO_H_LEG3_DRIVE:
    case NavPhase::GOTO_H_LEG3_OPEN_DRIVE:
    case NavPhase::GOTO_H_LEG4_DRIVE:
    case NavPhase::GOTO_H_FINAL_APP:
      return true;
    default:
      return false;
  }
}

static double haversine_m(double lat1, double lng1, double lat2, double lng2) {
  if (!isfinite(lat1) || !isfinite(lng1) || !isfinite(lat2) || !isfinite(lng2)) return NAN;
  constexpr double kEarthRadiusM = 6371000.0;
  constexpr double kDegToRad = PI / 180.0;
  const double phi1 = lat1 * kDegToRad;
  const double phi2 = lat2 * kDegToRad;
  const double dphi = (lat2 - lat1) * kDegToRad;
  const double dlambda = (lng2 - lng1) * kDegToRad;
  const double a = sin(dphi / 2.0) * sin(dphi / 2.0) +
                   cos(phi1) * cos(phi2) * sin(dlambda / 2.0) * sin(dlambda / 2.0);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return kEarthRadiusM * c;
}

static int32_t nav_distance_to_dest_mm(const AppState& state) {
  const double dist_m = haversine_m(state.nav.ref_lat, state.nav.ref_lng, state.nav.dest_lat, state.nav.dest_lng);
  if (!isfinite(dist_m) || dist_m < 0.0) return kMissingI32;
  const double mm = dist_m * 1000.0;
  if (!isfinite(mm)) return kMissingI32;
  const double clamped = (mm > 2000000000.0) ? 2000000000.0 : mm;
  return (int32_t)lround(clamped);
}

static String format_nav_dist_line(const AppState& state, const String& action) {
  if (action != "FORWARD") return "";
  if (!gps_trust_for_nav_distance(state)) return "";
  if (!is_goto_h_drive_phase(state.nav.current_phase_enum)) return "";
  const int32_t mm = nav_distance_to_dest_mm(state);
  if (mm == kMissingI32) return "";
  if (mm >= 10000) {
    const int32_t m = (int32_t)lround((double)mm / 1000.0);
    return String("dist:") + String((int)m) + "m";
  }
  return String("dist:") + String((int)mm);
}

static int32_t yaw_display_deg(const AppState& state) {
  if (!state.imu.valid || !isfinite(state.imu.yaw)) return kMissingI32;
  return (int32_t)lroundf(wrap_deg(state.imu.yaw));
}

static void display_oled_draw_dashboard(Adafruit_SSD1306& display, const AppState& state) {
  display.clearDisplay();

  // Column separators.
  display.drawLine(kSep1X, 0, kSep1X, display.height() - 1, SSD1306_WHITE);
  display.drawLine(kSep2X, 0, kSep2X, display.height() - 1, SSD1306_WHITE);

  // Left column: IMU calibration status.
  if (!s_cal_hidden) {
    draw_cal_row(display, 'g', state.imu.cal_gyro, kRow1Y);
    draw_cal_row(display, 'a', state.imu.cal_accel, kRow2Y);
    draw_cal_row(display, 'm', state.imu.cal_mag, kRow3Y);
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setFont(NULL);
  display.setCursor(kCalX + 2, kYawRateY);
  display.print("y:");
  const int32_t yaw_deg = s_cached_yaw_deg;
  if (yaw_deg == kMissingI32) {
    display.print("n/a");
  } else {
    display.print((int)yaw_deg);
  }

  // Center column: movement/status text.
  const String action = dashboard_action(state);
  const uint8_t pct = state.nav.motion_percent;
  const int col_x = kSep1X + 1;
  const int col_w = (kSep2X - kSep1X) - 2;
  const String dist_line = format_nav_dist_line(state, action);

  const bool goto_h_reached = (state.nav.current_phase_enum == NavPhase::GOTO_H_DONE) &&
                              (state.nav.target_key == "goto_h");

  if (goto_h_reached) {
    draw_centered_text_in_column(display, "STOP", col_x, col_w, kRow1Y, 1);
    draw_centered_text_in_column(display, "REACHED", col_x, col_w, kRow2Y, 1);
    draw_centered_text_in_column(display, "H", col_x, col_w, kRow3Y, 1);
  } else
  if (action == "FORWARD" || action == "BACKWARD") {
    draw_centered_text_in_column(display, action, col_x, col_w, kRow1Y, 1);
    draw_centered_text_in_column(display, String((int)pct), col_x, col_w, kRow2Y, 1);
    if (dist_line.length()) {
      draw_centered_text_in_column(display, dist_line, col_x, col_w, kRow3Y, 1);
    }
  } else if (action == "ROTATE_LEFT" || action == "ROTATE_RIGHT") {
    draw_centered_text_in_column(display, "ROTATE", col_x, col_w, kRow1Y, 1);
    const String dir = (action == "ROTATE_LEFT") ? "LEFT" : "RIGHT";
    draw_centered_text_in_column(display, with_percent(dir, pct), col_x, col_w, kRow2Y, 1);
    if (isfinite(state.nav.aim_yaw_deg)) {
      const int aim_deg = (int)lroundf(state.nav.aim_yaw_deg);
      draw_centered_text_in_column(display, String("AIM: ") + String(aim_deg), col_x, col_w, kRow3Y, 1);
    }
  } else if (action == "STEP_LEFT" || action == "STEP_RIGHT") {
    draw_centered_text_in_column(display, "STEP", col_x, col_w, kRow1Y, 1);
    const String dir = (action == "STEP_LEFT") ? "LEFT" : "RIGHT";
    draw_centered_text_in_column(display, with_percent(dir, pct), col_x, col_w, kRow2Y, 1);
  } else if (action == "STOP" || action == "IDLE") {
    draw_centered_text_in_column(display, action, col_x, col_w, kRow1Y, 2);
  } else if (action.length() > 0) {
    draw_centered_text_in_column(display, action, col_x, col_w, kRow1Y, 2);
  }

  // Right column: ultrasonic distances.
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setFont(NULL);
  display.setCursor(kDistX, kRow1Y);
  display.print("F:");
  display.print((int)s_cached_us_front_mm);
  display.setCursor(kDistX, kRow2Y);
  display.print("R:");
  display.print((int)s_cached_us_right_mm);
  display.setCursor(kDistX, kRow3Y);
  display.print("L:");
  display.print((int)s_cached_us_left_mm);
}

}  // namespace

// ---------------- public API ----------------

bool display_oled_begin(Adafruit_SSD1306& display){
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    return false;
  }

  display.clearDisplay();
  display.setRotation(0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setFont(NULL);
  display.setTextWrap(false);
  display.display();

  s_last_refresh_ms = millis();
  s_last_activity_ms = s_last_refresh_ms;
  s_display_is_on = true;
  s_last_message = "";
  s_last_dashboard_valid = false;
  s_last_dashboard_hash = 0;
  s_last_sensor_ms = 0;
  s_cached_yaw_deg = kMissingI32;
  s_cached_us_front_mm = 0;
  s_cached_us_right_mm = 0;
  s_cached_us_left_mm = 0;
  s_cal_complete_since_ms = 0;
  s_cal_hidden = false;
  return true;
}

void display_oled_show_message(Adafruit_SSD1306& display, const String& message){
  const bool messageChanged = message != s_last_message;
  const bool wasOff = !s_display_is_on;
  const uint32_t now = millis();

  if (messageChanged || wasOff){
    display.ssd1306_command(SSD1306_DISPLAYON);
    s_display_is_on = true;
    s_last_activity_ms = now;
  } else {
    return;
  }

  const int H = display.height();

  display.clearDisplay();

  String statusStr;
  String dataStr;
  split_status_and_data(message, &statusStr, &dataStr);

  const bool has_data = dataStr.length() > 0;
  const int total_h = has_data ? (kLineHeightPx + kLineSpacingPx + kLineHeightPx) : kLineHeightPx;
  int baseY = (H - total_h) / 2;
  if (baseY < 0) baseY = 0;

  draw_centered_text(display, statusStr, baseY, kTextSize);
  if (has_data) {
    draw_centered_text(display, dataStr, baseY + kLineHeightPx + kLineSpacingPx, kTextSize);
  }

  display.display();
  s_last_message = message;
}

void display_oled_update(Adafruit_SSD1306& display, const AppState& state){
  const uint32_t now = millis();
  if (now - s_last_refresh_ms < kMinRefreshMs) return;
  s_last_refresh_ms = now;

  if (s_display_is_on && (now - s_last_activity_ms >= kScreenTimeoutMs)){
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    s_display_is_on = false;
  }

  // Hide calibration rows once they remain fully complete for a while.
  const bool cal_complete =
      (state.imu.cal_gyro >= 3) && (state.imu.cal_accel >= 3) && (state.imu.cal_mag >= 3);
  if (cal_complete) {
    if (!s_cal_complete_since_ms) s_cal_complete_since_ms = now;
    if (!s_cal_hidden && (uint32_t)(now - s_cal_complete_since_ms) >= kCalHideDelayMs) {
      s_cal_hidden = true;
    }
  } else {
    s_cal_complete_since_ms = 0;
    s_cal_hidden = false;
  }

  // Update yaw + ultrasonic values at a fixed cadence (100ms).
  if (!s_last_sensor_ms || (uint32_t)(now - s_last_sensor_ms) >= kSensorRefreshMs) {
    s_last_sensor_ms = now;

    s_cached_yaw_deg = yaw_display_deg(state);
    s_cached_us_front_mm = (int32_t)state.us.front_mm;
    s_cached_us_right_mm = (int32_t)state.us.right_mm;
    s_cached_us_left_mm = (int32_t)state.us.left_mm;
  }

  const String action = dashboard_action(state);
  const String dist_line = format_nav_dist_line(state, action);

  // Hash the parts of AppState we render so we can avoid unnecessary I2C transfers.
  uint32_t h = 2166136261u;
  h = fnv1a_add_string(h, action);
  h = fnv1a_add_string(h, dist_line);
  h = fnv1a_add_u8(h, (uint8_t)state.nav.current_phase_enum);
  h = fnv1a_add_string(h, state.nav.target_key);
  h = fnv1a_add_u8(h, state.nav.motion_percent);
  h = fnv1a_add_u8(h, (uint8_t)(s_cal_hidden ? 1 : 0));
  if (!s_cal_hidden) {
    h = fnv1a_add_u8(h, state.imu.cal_gyro);
    h = fnv1a_add_u8(h, state.imu.cal_accel);
    h = fnv1a_add_u8(h, state.imu.cal_mag);
  }
  h = fnv1a_add_u32(h, (uint32_t)s_cached_yaw_deg);
  const int32_t aim_hash = ((action == "ROTATE_LEFT" || action == "ROTATE_RIGHT") && isfinite(state.nav.aim_yaw_deg))
                               ? (int32_t)lroundf(state.nav.aim_yaw_deg)
                               : kMissingI32;
  h = fnv1a_add_u32(h, (uint32_t)aim_hash);
  h = fnv1a_add_u32(h, (uint32_t)s_cached_us_front_mm);
  h = fnv1a_add_u32(h, (uint32_t)s_cached_us_right_mm);
  h = fnv1a_add_u32(h, (uint32_t)s_cached_us_left_mm);

  const bool changed = !s_last_dashboard_valid || (h != s_last_dashboard_hash);
  if (!s_display_is_on) {
    if (!changed) return;
    display.ssd1306_command(SSD1306_DISPLAYON);
    s_display_is_on = true;
  }

  if (!changed) return;
  s_last_activity_ms = now;
  s_last_dashboard_valid = true;
  s_last_dashboard_hash = h;

  display_oled_draw_dashboard(display, state);
  display.display();
}
