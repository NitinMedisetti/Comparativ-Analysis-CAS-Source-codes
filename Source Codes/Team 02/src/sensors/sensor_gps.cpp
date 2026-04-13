#include "sensor_gps.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>

#include "timebase.h"
#include "ui/ws_telemetry.h"

// GPS module integration:
// - Configures a u-blox receiver over UART using UBX messages (rate + NMEA filtering).
// - Parses incoming NMEA using TinyGPS++ and writes results into `AppState`.

// --- CET/CEST conversion helpers ---
static bool isLeapYear(int y){
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

static int lastDayOfMonth(int y, int m){
  static const int mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m == 2) return mdays[m-1] + (isLeapYear(y) ? 1 : 0);
  return mdays[m-1];
}

// Sakamoto's algorithm: 0=Sunday .. 6=Saturday
static int dayOfWeek(int y, int m, int d){
  static const int t[12]={0,3,2,5,0,3,5,1,4,6,2,4};
  if (m < 3) y -= 1;
  return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

static int lastSunday(int y, int m){
  const int ld = lastDayOfMonth(y, m);
  const int w = dayOfWeek(y, m, ld); // 0=Sun
  return ld - w; // if last day is Sun (w=0), it's the last Sunday; else subtract weekday
}

// EU DST rules: CEST (UTC+2) between last Sunday in March 01:00 UTC and last Sunday in Oct 01:00 UTC
static bool isCESTPeriodUTC(int y, int m, int d, int h){
  if (m < 3 || m > 10) return false;        // Jan-Feb, Nov-Dec => CET
  if (m > 3 && m < 10) return true;         // Apr-Sep => CEST
  const int mMarLastSun = lastSunday(y, 3);
  const int mOctLastSun = lastSunday(y, 10);
  if (m == 3){
    if (d > mMarLastSun) return true;
    if (d < mMarLastSun) return false;
    // On the transition day: after 01:00 UTC it's CEST
    return h >= 1;
  }
  // m == 10
  if (d < mOctLastSun) return true;
  if (d > mOctLastSun) return false;
  // On the transition day: before 01:00 UTC it's still CEST
  return h < 1;
}

static void utcToCentralEurope(int y, int m, int d, int hh, int mm, int ss,
                               int &oy, int &om, int &od, int &oh, int &omin, int &os,
                               bool &outIsCEST){
  outIsCEST = isCESTPeriodUTC(y, m, d, hh);
  int offset = outIsCEST ? 2 : 1; // hours
  oy=y; om=m; od=d; oh=hh; omin=mm; os=ss;
  oh += offset;
  if (oh >= 24){
    oh -= 24;
    od += 1;
    int ld = lastDayOfMonth(oy, om);
    if (od > ld){
      od = 1;
      om += 1;
      if (om > 12){ om = 1; oy += 1; }
    }
  }
}

static void sendUBX(uint8_t cls, uint8_t id, const uint8_t* pl, uint16_t len){
  uint8_t hdr[4]={cls,id,(uint8_t)(len&0xFF),(uint8_t)(len>>8)};
  uint8_t cka=0, ckb=0;
  for (uint8_t i=0; i<sizeof(hdr); ++i){ cka+=hdr[i]; ckb+=cka; }
  for (uint16_t i=0; i<len; ++i){ cka+=pl[i]; ckb+=cka; }
  Serial2.write(0xB5); Serial2.write(0x62);
  Serial2.write(hdr,4);
  if(len) Serial2.write(pl,len);
  Serial2.write(cka); Serial2.write(ckb);
  Serial2.flush();
}

static void gpsCfgPort38400(){
  uint8_t pl[20]={0};
  pl[0]=1; // UART1
  pl[4]=0xD0; pl[5]=0x08; // 0x000008D0 -> 8N1
  pl[8]=0x00; pl[9]=0x96; pl[10]=0x00; pl[11]=0x00; // 38400 baud
  pl[12]=0x03; pl[13]=0x00; // UBX+NMEA in
  pl[14]=0x03; pl[15]=0x00; // UBX+NMEA out
  sendUBX(0x06,0x00,pl,sizeof(pl));
}

static void gpsCfgRate5Hz(){
  uint8_t pl[6]; uint16_t meas=200, nav=1, tref=1;
  pl[0]=meas&0xFF; pl[1]=meas>>8; pl[2]=nav&0xFF; pl[3]=nav>>8; pl[4]=tref&0xFF; pl[5]=tref>>8;
  sendUBX(0x06,0x08,pl,sizeof(pl));
}

static void gpsCfgMsgRate(uint8_t msgId, uint8_t uartRate){
  uint8_t pl[8]={0xF0,msgId,0x00,uartRate,0x00,0x00,0x00,0x00};
  sendUBX(0x06,0x01,pl,sizeof(pl));
}

static void gpsCfgDisableMessages(){
  const uint8_t disableList[]={0x01,0x02,0x03,0x05,0x08}; // GLL, GSA, GSV, VTG, ZDA
  for (uint8_t i=0; i<sizeof(disableList); ++i){
    gpsCfgMsgRate(disableList[i], 0);
  }
  gpsCfgMsgRate(0x00, 1); // GGA
  gpsCfgMsgRate(0x04, 1); // RMC
}

void sensor_gps_begin(int rx_pin, int tx_pin){
  Serial2.begin(GPS_UART_BAUD, SERIAL_8N1, rx_pin, tx_pin);
  delay(100);
  gpsCfgPort38400();
  Serial2.end();
  delay(100);
  Serial2.begin(38400, SERIAL_8N1, rx_pin, tx_pin);
  delay(50);
  gpsCfgRate5Hz();
  gpsCfgDisableMessages();
}

void sensor_gps_poll(TinyGPSPlus& parser){
  while (Serial2.available()){
    int c = Serial2.read();
    if (c=='\r') continue;
    parser.encode((char)c);
  }
}

void sensor_gps_update_state(TinyGPSPlus& parser, AppState& state){
  const bool updated =
      parser.location.isUpdated() || parser.altitude.isUpdated() || parser.speed.isUpdated() ||
      parser.course.isUpdated() || parser.hdop.isUpdated() || parser.satellites.isUpdated() ||
      parser.time.isUpdated() || parser.date.isUpdated();
  if (updated) {
    state.gps.stamp.t_us = now_us();
    state.gps.stamp.seq += 1;
  }

  state.gps.valid = parser.location.isValid();
  state.gps.lat = state.gps.valid ? parser.location.lat() : NAN;
  state.gps.lng = state.gps.valid ? parser.location.lng() : NAN;
  state.gps.alt_m = parser.altitude.isValid() ? parser.altitude.meters() : NAN;
  state.gps.speed_kmh = parser.speed.isValid() ? parser.speed.kmph() : NAN;
  state.gps.heading_deg = parser.course.isValid() ? parser.course.deg() : NAN;
  state.gps.hdop = parser.hdop.isValid() ? parser.hdop.hdop() : NAN;
  state.gps.sats = parser.satellites.value();
  state.gps.age_ms = parser.location.isValid() ? parser.location.age() : 0;

  if (parser.time.isValid() && parser.date.isValid()){
    int y = (int)parser.date.year();
    int mo = parser.date.month();
    int da = parser.date.day();
    int hh = parser.time.hour();
    int mi = parser.time.minute();
    int ss = parser.time.second();
    int oy, om, od, oh, omin, os;
    bool isCEST=false;
    utcToCentralEurope(y, mo, da, hh, mi, ss, oy, om, od, oh, omin, os, isCEST);

    char tbuf[24];
    snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d %s", oh, omin, os, isCEST?"CEST":"CET");
    state.gps.utc = String(tbuf);

    char dbuf[16];
    snprintf(dbuf, sizeof(dbuf), "%04d-%02d-%02d", oy, om, od);
    state.gps.date = String(dbuf);
  } else {
    state.gps.utc = "null";
    state.gps.date = "null";
  }

  if (!ui::telemetry_ws_has_clients()) {
    return;
  }

  // GPS is configured for ~5Hz updates; do not spam identical frames every loop tick.
  static constexpr uint32_t kGpsTelemetryMinIntervalMs = 200;
  static uint32_t last_send_ms = 0;
  const uint32_t now_ms = millis();
  if (!updated) {
    return;
  }
  if (last_send_ms != 0 && (uint32_t)(now_ms - last_send_ms) < kGpsTelemetryMinIntervalMs) {
    return;
  }
  last_send_ms = now_ms;

  JsonDocument doc;
  doc["type"] = "gps";
  doc["lat"] = state.gps.lat;
  doc["lng"] = state.gps.lng;
  doc["hdop"] = state.gps.hdop;
  doc["sats"] = state.gps.sats;
  ui::telemetry_ws_send(doc);
}
