/*
  GpsManager.cpp
  Disclaimer:
  This GPS manager was changed several times in due course of development and therefore includes much more funcitons tha are currently used and needed by the main code.
*/

#include "GpsManager.h"


// -----------------------------------------------------------------------------
// Convenience helpers to keep application .ino files small.
// -----------------------------------------------------------------------------

GpsManager::CampusFrame GpsManager::thdCampusFrame()
{
  CampusFrame f;
  // define local campus frame y+ is aligned to buildings and when standing at Post in H direction.
  f.originLat  = 48.829653;
  f.originLon  = 12.956594;
  f.cornerXLat = 48.830573;
  f.cornerXLon = 12.954024;
  f.cornerYLat = 48.828579;
  f.cornerYLon = 12.955702;
  return f;
}

void GpsManager::thdTouchUpOrigin(double& latDeg, double& lonDeg)
{
  latDeg = 48.829932;
  lonDeg = 12.954907;
}

GpsManager::Config GpsManager::makeThdCampusDefaultConfig()
{
  Config cfg;

  // UART (wie bisher im .ino)
  cfg.gpsBaud  = 9600;
  cfg.gpsRxPin = 16;
  cfg.gpsTxPin = 17;

  // Campus-Frame (wie bisher im .ino)
  const CampusFrame f = thdCampusFrame();
  cfg.campusFrameEnabled = true;
  cfg.campusOriginLat  = f.originLat;
  cfg.campusOriginLon  = f.originLon;
  cfg.campusCornerXLat = f.cornerXLat;
  cfg.campusCornerXLon = f.cornerXLon;
  cfg.campusCornerYLat = f.cornerYLat;
  cfg.campusCornerYLon = f.cornerYLon;

  // optional complex filters (in testing state, be careful)
  cfg.enableQualityGating       = false;
  cfg.enableLatencyCompensation = false;
  cfg.enableAlphaBetaFilter     = false;
  cfg.enableAdaptiveNoise       = false;
  cfg.enableOutlierGating       = false;
  cfg.enableZUPT                = false;

  // quality params
  cfg.maxHdop       = 1.5;
  cfg.minSatellites = 7;
  cfg.maxFixAgeMs   = 1500;
  cfg.zuptSpeedThreshMps = 0.05;

  // optional and recomended basic filters
  cfg.enableMedianFilter = true;
  cfg.medianWindowSize   = 5;

  cfg.enableMeanFilter = false;
  cfg.meanWindowSize   = 5;

  return cfg;
}

bool GpsManager::begin()
{
  const Config cfg = makeThdCampusDefaultConfig();
  return begin(Serial2, cfg);
}


bool GpsManager::setTouchUpOriginThdDefault(bool resetFilterAfter)
{//set origin to saved default value
  double lat, lon;
  thdTouchUpOrigin(lat, lon);
  const bool ok = setTouchUpOriginLatLon(lat, lon);
  if (ok && resetFilterAfter) {
    resetFilter();
  }
  return ok;
}

void GpsManager::buildTelemetry(Telemetry& t, uint32_t freshMaxMs, float sentinel)
{
  if (freshMaxMs == 0) {
    freshMaxMs = _cfg.maxFixAgeMs;
  }

  // XY
  t.haveXY = getCampusXY(t.x_m, t.y_m);

  // LLA
  t.llValid = getCurrentLatLon(t.latDeg, t.lonDeg);

  // Quality
  t.qualOk = getLastQuality(t.hdop, t.sats, t.ageMs);

  // Freshness / validity (wie bisher im .ino)
  const bool finiteLL = isfinite(t.latDeg) && isfinite(t.lonDeg);
  t.freshFix = t.llValid && t.qualOk && finiteLL && (t.ageMs <= freshMaxMs);

  // Sentinel outputs (JSON-sicher)
  t.X_out   = t.haveXY  ? (double)t.x_m : (double)sentinel;
  t.Y_out   = t.haveXY  ? (double)t.y_m : (double)sentinel;
  t.LAT_out = t.freshFix ? (double)t.latDeg : (double)sentinel;
  t.LON_out = t.freshFix ? (double)t.lonDeg : (double)sentinel;

  t.HDOP_out = t.qualOk ? (float)t.hdop : sentinel;
  t.Sats_out = t.qualOk ? (float)t.sats : sentinel;
  t.Age_out  = t.qualOk ? (float)t.ageMs : sentinel;

  t.Fix_out = t.freshFix ? 1 : 0;

  const FixStatus st = getFixStatus();
  t.fixStatusCode = static_cast<uint8_t>(st);

  buildDebugString(t.debug, sizeof(t.debug));
}

GpsManager::OriginSetStatus GpsManager::trySetTouchUpOriginFromCurrentFix(uint32_t freshMaxMs, bool resetFilterAfter)
{// change origin to current value
  Telemetry t;
  buildTelemetry(t, freshMaxMs, 0.0f);

  if (!t.freshFix) {
    return OriginSetStatus::NO_FRESH_FIX;
  }

  const bool ok = setTouchUpOriginLatLon(t.latDeg, t.lonDeg);
  if (!ok) {
    return OriginSetStatus::FRAME_NOT_READY;
  }

  if (resetFilterAfter) {
    resetFilter();
  }
  return OriginSetStatus::SET_OK;
}

void GpsManager::printTelemetry(Stream& out, const Telemetry& t,
                                uint8_t xyPrec, uint8_t hdopPrec, uint8_t llaPrec) const
{// Serial prints for debugging
  out.print(F("X: "));
  out.print(t.X_out, xyPrec);
  out.print(F(" Y: "));
  out.print(t.Y_out, xyPrec);

  out.print(F(" | HDOP: "));
  out.print(t.HDOP_out, hdopPrec);
  out.print(F(" Sats: "));
  out.print((int)t.Sats_out);
  out.print(F(" Age[ms]: "));
  out.print((uint32_t)t.Age_out);

  out.print(F(" | LAT: "));
  out.print(t.LAT_out, llaPrec);
  out.print(F(" LON: "));
  out.print(t.LON_out, llaPrec);

  out.print(F(" | Fix: "));
  out.print(t.Fix_out);

  const FixStatus st = static_cast<FixStatus>(t.fixStatusCode);
  out.print(F(" | FixStatus="));
  out.print(getFixStatusString(st));

  out.print(F(" | Debug="));
  out.println(t.debug);
}

void GpsManager::printTelemetry(Stream& out, uint32_t freshMaxMs, float sentinel)
{
  Telemetry t;
  buildTelemetry(t, freshMaxMs, sentinel);
  printTelemetry(out, t);
}

namespace {
  constexpr double EARTH_R  = 6371000.0;            // [m]
  constexpr double DEG2RAD  = 0.017453292519943295; // pi/180
  constexpr double RAD2DEG  = 57.29577951308232;
  constexpr double EPS_NORM = 1e-6;

  inline double hypot2(double x, double y) { return std::sqrt(x * x + y * y); }

  template<typename T>
  inline T clamp(T v, T lo, T hi) { return std::max(lo, std::min(v, hi)); }

  // --- Date/Time helpers for DST calculation (Europe/Berlin) ---
  inline bool isLeap(int Y) {
    return (Y % 4 == 0) && ((Y % 100) != 0 || (Y % 400 == 0));
  }

  inline int daysInMonth(int Y, int M) {
    static const uint8_t d[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    return (M == 2 && isLeap(Y)) ? 29 : d[M - 1];
  }

  // Day of week, 0 = Sunday ... 6 = Saturday
  inline int dayOfWeek(int Y, int M, int D) {
    static int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
    if (M < 3) Y -= 1;
    return (Y + Y/4 - Y/100 + Y/400 + t[M-1] + D) % 7;
  }

  // Last Sunday of given month
  inline int lastSunday(int Y, int M) {
    int last = daysInMonth(Y, M);
    return last - dayOfWeek(Y, M, last);
  }
}

// Public API ------------------------------------------------------------------

bool GpsManager::begin(HardwareSerial& port, const Config& cfg)
{
  _serial = &port;
  _cfg    = cfg;  // komplette Config in das Objekt kopieren

  // Falls GOOD/WARN-Grenzen nicht gesetzt wurden, sinnvolle Defaults wählen
  if (_cfg.hdopGood <= 0.0) {
    _cfg.hdopGood = 0.9;
  }
  if (_cfg.hdopWarn <= 0.0) {
    _cfg.hdopWarn = _cfg.maxHdop;
  }
  if (_cfg.satsGood == 0) {
    _cfg.satsGood = _cfg.minSatellites + 2;
  }
  if (_cfg.satsWarn == 0) {
    _cfg.satsWarn = _cfg.minSatellites;
  }

  // Filter-Pipeline ist jetzt nur noch Median/Mean
  _filterActive =
      _cfg.enableMedianFilter ||
      _cfg.enableMeanFilter;

  _histCount = 0;

  _serial->begin(_cfg.gpsBaud, SERIAL_8N1, _cfg.gpsRxPin, _cfg.gpsTxPin);
  _configureNeo6m5Hz();
  _campusFrameEnabled = _cfg.campusFrameEnabled;
  if (_campusFrameEnabled) {
    _initCampusFrame(_cfg.campusOriginLat, _cfg.campusOriginLon,
                     _cfg.campusCornerXLat, _cfg.campusCornerXLon,
                     _cfg.campusCornerYLat, _cfg.campusCornerYLon);
  } else {
    _campusAxesReady = false;
  }

  _stateValid   = false;
  _x = _y = 0.0;
  _vx = _vy = 0.0;
  _lastUpdateMs = millis();

  return true;
}


void GpsManager::poll()
{
  if (!_serial) return;
  while (_serial->available() > 0) {
    _gps.encode(_serial->read());
    // Process immediately when a new location arrives (keeps latency low)
    if (_gps.location.isUpdated()) {
      _ingestIfUpdated();
    }
  }
}

bool GpsManager::hasValidLocation()
{
  return _gps.location.isValid();
}

void GpsManager::_configureNeo6m5Hz()
{// this configures the gps module to provide updates every 200 ms
  if (!_serial) return;

  // ---------------------------------------------------------------------------
  // 0) Reset to factory defaults (clear + load)
  //    UBX-CFG-CFG (Class 0x06, ID 0x09), length = 13
  //    clearMask = 0xFFFFFFFF, saveMask = 0x00000000, loadMask = 0xFFFFFFFF
  //    deviceMask = 0x01 (BBR)
  //
  // Checksum (CK_A, CK_B) for this exact payload: 0x15, 0x7E
  // If you want to target BBR + Flash, change deviceMask to 0x03 and checksum to 0x17, 0x80
  // ---------------------------------------------------------------------------

  const uint8_t cfgFactoryReset_BBR[] = {
    0xB5, 0x62,             // Sync chars
    0x06, 0x09,             // Class = CFG, ID = CFG
    0x0D, 0x00,             // Length = 13
    0xFF, 0xFF, 0xFF, 0xFF, // clearMask
    0x00, 0x00, 0x00, 0x00, // saveMask
    0xFF, 0xFF, 0xFF, 0xFF, // loadMask
    0x01,                   // deviceMask (BBR)
    0x15, 0x7E              // Checksum (CK_A, CK_B)
  };

  for (size_t i = 0; i < sizeof(cfgFactoryReset_BBR); ++i) {
    _serial->write(cfgFactoryReset_BBR[i]);
    delay(5);
  }
  _serial->flush();
  delay(500); // give the module time to apply defaults

  // ---------------------------------------------------------------------------
  // 1) UBX-CFG-RATE: 5 Hz (200 ms)
  // ---------------------------------------------------------------------------

  const uint8_t cfgRate5Hz[] = {
    0xB5, 0x62,             // Sync chars
    0x06, 0x08,             // Class = CFG, ID = RATE
    0x06, 0x00,             // Length = 6
    0xC8, 0x00,             // rate  = 200 ms
    0x01, 0x00,             // navRate   = 1
    0x01, 0x00,             // timeRef   = 1 (GPS)
    0xDE, 0x6A              // Checksum (CK_A, CK_B)
  };

  for (size_t i = 0; i < sizeof(cfgRate5Hz); ++i) {
    _serial->write(cfgRate5Hz[i]);
    delay(5);
  }
  _serial->flush();
  delay(200); // Modul kurz Zeit geben, die Einstellung zu übernehmen
}



bool GpsManager::getCurrentLatLon(double& latDeg, double& lonDeg)
{
  if (!_gps.location.isValid()) return false;
  latDeg = _gps.location.lat();
  lonDeg = _gps.location.lng();
  return true;
}

bool GpsManager::latLonToLocalXY(double latDeg, double lonDeg,
                                 float& x_m, float& y_m) const
{// changes lat and long values into local frame x and y values
  // Convenience wrapper around the existing campus-frame logic.
  return toCampusXY(latDeg, lonDeg, x_m, y_m);
}

bool GpsManager::toCampusXY(double latDeg, double lonDeg, float& x_m, float& y_m) const
{
  if (!_campusFrameEnabled || !_campusAxesReady) return false;

  double E, N;
  _toEN(latDeg, lonDeg, E, N);

  // Raw projection
  const double xr = E * _exx + N * _exy;
  const double yr = E * _eyx + N * _eyy;

  // Origin offset
  x_m = static_cast<float>(xr - _originOffsetX_m);
  y_m = static_cast<float>(yr - _originOffsetY_m);
  return true;
}

bool GpsManager::getCampusXY(float& x_m, float& y_m)
{// main funciton to get the Modules Position on the campus in x and y values according to the local frame.
  if (!_campusFrameEnabled || !_campusAxesReady) return false;

  
  if (_stateValid) {
    x_m = static_cast<float>(_x);
    y_m = static_cast<float>(_y);
    return true;
  }

  // Fallback
  if (!_gps.location.isValid()) return false;
  const double lat = _gps.location.lat();
  const double lon = _gps.location.lng();
  return toCampusXY(lat, lon, x_m, y_m);
}

// Optional helpers -------------------------------------------------------------

bool GpsManager::getLastQuality(double& hdop, int& sats, uint32_t& ageMs) const
{
  if (!_stateValid && !_gps.location.isValid()) return false;
  hdop  = _lastHdop;
  sats  = _lastSats;
  ageMs = _lastAgeMs;
  return true;
  return true;
}

bool GpsManager::getLastCogDeg(double& cogDeg) const
{
  if (!_gps.course.isValid()) return false;
  cogDeg = _lastCogDeg;
  return !std::isnan(_lastCogDeg);
}

bool GpsManager::getLastSogMps(double& sogMps) const
{
  if (!_gps.speed.isValid()) return false;
  sogMps = _lastSogMps;
  return true;
}

void GpsManager::resetFilter()
{
  _stateValid = false;
  _x = _y = 0.0;
  _vx = _vy = 0.0;
  _lastUpdateMs = millis();
  _histCount = 0;
}

// --- GNSS fix quality status & debug string ----------------------------------

GpsManager::FixStatus GpsManager::getFixStatus() const
{
  // 1) No data processed yet
  if (_lastAgeMs == 0 && _lastSats == 0 && !_stateValid) {
    return FixStatus::NO_DATA;
  }

  // 2) BASIC BAD CHECKS (harte Grenzen für Status-Bewertung)
  const bool okHdopBasic = (_lastHdop <= _cfg.maxHdop);
  const bool okSatsBasic = (_lastSats >= static_cast<int>(_cfg.minSatellites));
  const bool okAgeBasic  = (_lastAgeMs <= _cfg.maxFixAgeMs);

  if (!(okHdopBasic && okSatsBasic && okAgeBasic)) {
    return FixStatus::BAD;
  }

  // 3) GOOD / WARN thresholds
  const bool isGood =
      (_lastHdop <= _cfg.hdopGood) &&
      (_lastSats >= static_cast<int>(_cfg.satsGood));

  if (isGood) {
    return FixStatus::GOOD;
  }

  const bool isWarn =
      (_lastHdop <= _cfg.hdopWarn) &&
      (_lastSats >= static_cast<int>(_cfg.satsWarn));

  if (isWarn) {
    return FixStatus::WARN;
  }

  return FixStatus::BAD;
}


const char* GpsManager::getFixStatusString(FixStatus s) const
{
  switch (s) {
    case FixStatus::NO_DATA: return "NO_DATA";
    case FixStatus::BAD:     return "BAD";
    case FixStatus::WARN:    return "WARN";
    case FixStatus::GOOD:    return "GOOD";
    default:                 return "UNKNOWN";
  }
}

void GpsManager::buildDebugString(char* buf, size_t len) const
{
  if (!buf || len == 0) return;

  const FixStatus st   = getFixStatus();
  const char*     name = getFixStatusString(st);

  double    hdop = 0.0;
  int       sats = 0;
  uint32_t  age  = 0;
  const bool haveQ = getLastQuality(hdop, sats, age);

  double cog = 0.0;
  double sog = 0.0;
  const bool haveCog = getLastCogDeg(cog);
  const bool haveSog = getLastSogMps(sog);

  if (!haveQ) {
    snprintf(buf, len, "status=%s; no quality data", name);
    return;
  }

  if (haveCog && haveSog) {
    snprintf(buf, len,
             "status=%s; HDOP=%.2f; sats=%d; age=%lums; COG=%.1fdeg; SOG=%.2fm/s",
             name, hdop, sats, static_cast<unsigned long>(age), cog, sog);
  } else if (haveSog) {
    snprintf(buf, len,
             "status=%s; HDOP=%.2f; sats=%d; age=%lums; SOG=%.2fm/s",
             name, hdop, sats, static_cast<unsigned long>(age), sog);
  } else {
    snprintf(buf, len,
             "status=%s; HDOP=%.2f; sats=%d; age=%lums",
             name, hdop, sats, static_cast<unsigned long>(age));
  }
}

// --- Local time helpers: UTC -> CET/CEST -------------------------------------

bool GpsManager::_utcToCet(int& y, int& m, int& d, int& h, int& mi, int& s) const
{ // time conversion from utc to cet
  TinyGPSPlus& gps = const_cast<TinyGPSPlus&>(_gps);

  if (!(gps.date.isValid() && gps.time.isValid())) {
    return false;
  }

  y  = gps.date.year();
  m  = gps.date.month();
  d  = gps.date.day();
  h  = gps.time.hour();
  mi = gps.time.minute();
  s  = gps.time.second(); // UTC

  bool cest = false;
  if (m < 3 || m > 10) {
    cest = false; // Jan, Feb, Nov, Dec -> CET
  } else if (m > 3 && m < 10) {
    cest = true;  // Apr..Sep -> CEST
  } else if (m == 3) {
    int ls = lastSunday(y, 3);
    cest = (d > ls) || (d == ls && h >= 1);
  } else { // m == 10
    int ls = lastSunday(y, 10);
    cest = (d < ls) || (d == ls && h < 1);
  }

  int offset = cest ? 2 : 1;
  h += offset;

  if (h >= 24) {
    h -= 24;
    d += 1;
    int dim = daysInMonth(y, m);
    if (d > dim) {
      d = 1;
      m += 1;
      if (m > 12) {
        m = 1;
        y += 1;
      }
    }
  }
  return true;
}

// wrapper to get time date easily:
uint16_t GpsManager::getYearCET() const
{
  int y, m, d, h, mi, s;
  return _utcToCet(y, m, d, h, mi, s) ? static_cast<uint16_t>(y) : 0;
}

uint8_t GpsManager::getMonthCET() const
{
  int y, m, d, h, mi, s;
  return _utcToCet(y, m, d, h, mi, s) ? static_cast<uint8_t>(m) : 0;
}

uint8_t GpsManager::getDayCET() const
{
  int y, m, d, h, mi, s;
  return _utcToCet(y, m, d, h, mi, s) ? static_cast<uint8_t>(d) : 0;
}

uint8_t GpsManager::getHourCET() const
{
  int y, m, d, h, mi, s;
  return _utcToCet(y, m, d, h, mi, s) ? static_cast<uint8_t>(h) : 0;
}

uint8_t GpsManager::getMinuteCET() const
{
  int y, m, d, h, mi, s;
  return _utcToCet(y, m, d, h, mi, s) ? static_cast<uint8_t>(mi) : 0;
}

uint8_t GpsManager::getSecondCET() const
{
  int y, m, d, h, mi, s;
  return _utcToCet(y, m, d, h, mi, s) ? static_cast<uint8_t>(s) : 0;
}

// Private helpers --------------------------------------------------------------

void GpsManager::_initCampusFrame(double lat0, double lon0,
                                  double cxLat, double cxLon,
                                  double cyLat, double cyLon)
{
  _campusLat0 = lat0;
  _campusLon0 = lon0;

  const double lat0r = lat0 * DEG2RAD;
  _kx_m_per_deg = DEG2RAD * EARTH_R * std::cos(lat0r);
  _ky_m_per_deg = DEG2RAD * EARTH_R;

  auto toEN_local = [&](double la, double lo, double& E, double& N) {
    E = (lo - _campusLon0) * _kx_m_per_deg;
    N = (la - _campusLat0) * _ky_m_per_deg;
  };

  double vXx, vXy, vYx, vYy;
  toEN_local(cxLat, cxLon, vXx, vXy);
  toEN_local(cyLat, cyLon, vYx, vYy);

  const double vXnorm = hypot2(vXx, vXy);
  if (vXnorm < EPS_NORM) {
    _campusAxesReady = false;
    _exx = 1.0; _exy = 0.0;
    _eyx = 0.0; _eyy = 1.0;
    _xmax_m = 0.0; _ymax_m = 0.0;
    return;
  }
  _exx = vXx / vXnorm;
  _exy = vXy / vXnorm;

  const double dot_vY_ex = vYx * _exx + vYy * _exy;
  double vYpx = vYx - dot_vY_ex * _exx;
  double vYpy = vYy - dot_vY_ex * _exy;

  double vYpnorm = hypot2(vYpx, vYpy);
  if (vYpnorm < EPS_NORM) {
    vYpx = -_exy;
    vYpy =  _exx;
    vYpnorm = hypot2(vYpx, vYpy);
  }

  _eyx = vYpx / vYpnorm;
  _eyy = vYpy / vYpnorm;

  _xmax_m = vXnorm;
  _ymax_m = vYpnorm;

  _campusAxesReady = true;
}

void GpsManager::_toEN(double latDeg, double lonDeg, double& E_m, double& N_m) const
{
  E_m = (lonDeg - _campusLon0) * _kx_m_per_deg;
  N_m = (latDeg - _campusLat0) * _ky_m_per_deg;
}

bool GpsManager::_qualityPass(double hdop, int sats, uint32_t ageMs) const
{
  (void)hdop;
  (void)sats;
  (void)ageMs;
  return true;
}

void GpsManager::_applyLatencyComp(double& E_m, double& N_m,
                                   double sog_mps, double cog_deg, uint32_t ageMs) const
{
  // currently not used. Saved for later.
  if (!_cfg.enableLatencyCompensation) return;
  if (!(sog_mps > 0.0) || std::isnan(cog_deg)) return;

  const double dt = ageMs * 1e-3;
  const double cog_rad = cog_deg * DEG2RAD;
  const double dE = sog_mps * dt * std::sin(cog_rad);
  const double dN = sog_mps * dt * std::cos(cog_rad);
  E_m += dE;
  N_m += dN;
}

void GpsManager::_filterUpdate(double zX, double zY, double dt, double hdop, double sog_mps)
{
  // currently not used. Saved for later.
  (void)zX;
  (void)zY;
  (void)dt;
  (void)hdop;
  (void)sog_mps;
}


// For filters:
void GpsManager::_pushSample(double x, double y)
{
  if (_histCount < MAX_FILTER_WINDOW) {
    _histX[_histCount] = x;
    _histY[_histCount] = y;
    _histCount++;
  } else {
    for (uint8_t i = 1; i < MAX_FILTER_WINDOW; ++i) {
      _histX[i - 1] = _histX[i];
      _histY[i - 1] = _histY[i];
    }
    _histX[MAX_FILTER_WINDOW - 1] = x;
    _histY[MAX_FILTER_WINDOW - 1] = y;
  }
}

void GpsManager::_computeMean(double& fx, double& fy) const
{
  if (!_cfg.enableMeanFilter || _histCount == 0) {
    fx = _lastMeasX;
    fy = _lastMeasY;
    return;
  }

  uint8_t window = _cfg.meanWindowSize;
  if (window == 0) window = 1;
  if (window > MAX_FILTER_WINDOW) window = MAX_FILTER_WINDOW;
  if (window > _histCount) window = _histCount;

  double sumX = 0.0;
  double sumY = 0.0;
  const uint8_t start = static_cast<uint8_t>(_histCount - window);
  for (uint8_t i = start; i < _histCount; ++i) {
    sumX += _histX[i];
    sumY += _histY[i];
  }
  fx = sumX / window;
  fy = sumY / window;
}

void GpsManager::_computeMedian(double& fx, double& fy) const
{
  if (!_cfg.enableMedianFilter || _histCount == 0) {
    fx = _lastMeasX;
    fy = _lastMeasY;
    return;
  }

  uint8_t window = _cfg.medianWindowSize;
  if (window == 0) window = 1;
  if (window > MAX_FILTER_WINDOW) window = MAX_FILTER_WINDOW;
  if (window > _histCount) window = _histCount;

  double tmpX[MAX_FILTER_WINDOW];
  double tmpY[MAX_FILTER_WINDOW];

  const uint8_t start = static_cast<uint8_t>(_histCount - window);
  for (uint8_t i = 0; i < window; ++i) {
    tmpX[i] = _histX[start + i];
    tmpY[i] = _histY[start + i];
  }

  std::sort(tmpX, tmpX + window);
  std::sort(tmpY, tmpY + window);

  const uint8_t mid = window / 2;
  if (window % 2) {
    fx = tmpX[mid];
    fy = tmpY[mid];
  } else {
    fx = 0.5 * (tmpX[mid - 1] + tmpX[mid]);
    fy = 0.5 * (tmpY[mid - 1] + tmpY[mid]);
  }
}

bool GpsManager::setTouchUpOriginLatLon(double latDeg, double lonDeg)
{
  if (!_campusFrameEnabled || !_campusAxesReady) return false;

  double E, N;
  _toEN(latDeg, lonDeg, E, N);

  const double xr = E * _exx + N * _exy;
  const double yr = E * _eyx + N * _eyy;

  _originOffsetX_m = xr;
  _originOffsetY_m = yr;

  return true;
}

void GpsManager::_ingestIfUpdated()
{
  if (!_gps.location.isValid()) return;

  // Read GNSS quality and kinematics
  _lastHdop = _gps.hdop.isValid() ? (_gps.hdop.value() * 0.01) : 99.0;
  _lastSats = _gps.satellites.isValid() ? (int)_gps.satellites.value() : 0;
  _lastAgeMs = _gps.location.age();
  _lastCogDeg = _gps.course.isValid() ? _gps.course.deg() : NAN;
  _lastSogMps = _gps.speed.isValid()  ? _gps.speed.mps() : 0.0;

  
  (void)_qualityPass(_lastHdop, _lastSats, _lastAgeMs);

  // Measurement: lat/lon -> EN (meters) relativ to Origin
  const double lat = _gps.location.lat();
  const double lon = _gps.location.lng();

  double E, N;
  _toEN(lat, lon, E, N);

  _applyLatencyComp(E, N, _lastSogMps, _lastCogDeg, _lastAgeMs);


  const double zX = E * _exx + N * _exy - _originOffsetX_m;
  const double zY = E * _eyx + N * _eyy - _originOffsetY_m;

  _lastMeasX = zX;
  _lastMeasY = zY;

  _lastUpdateMs = millis();

  _pushSample(zX, zY);

  if (!_filterActive) {
    _x = zX;
    _y = zY;
    _vx = 0.0;
    _vy = 0.0;
    _stateValid = true;
    return;
  }

  if (!_stateValid) {
    _stateValid = true;
  }

  // Starting point
  double fx = zX;
  double fy = zY;


  if (_cfg.enableMedianFilter) {
    _computeMedian(fx, fy);
  }
  if (_cfg.enableMeanFilter) {
    _computeMean(fx, fy);
  }

  _x = fx;
  _y = fy;
  _vx = 0.0;
  _vy = 0.0;
}
