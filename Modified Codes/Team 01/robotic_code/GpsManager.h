#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <cmath>
#include <algorithm>

class GpsManager {
public:
  struct Config {
    // UART configuration
    uint32_t gpsBaud  = 9600;
    int      gpsRxPin = 16;   // GPS TX -> this RX
    int      gpsTxPin = 17;   // GPS RX -> this TX

    // Campus-aligned local frame (immutable during runtime)
    bool   campusFrameEnabled = false;
    double campusOriginLat    = 0.0;
    double campusOriginLon    = 0.0;
    double campusCornerXLat   = 0.0;
    double campusCornerXLon   = 0.0;
    double campusCornerYLat   = 0.0;
    double campusCornerYLon   = 0.0;

    // opt filters
    bool enableQualityGating       = false; // GNSS-quality checks (HDOP/Sats/Age)
    bool enableLatencyCompensation = false; // propagate fix by Age with COG/SOG
    bool enableAlphaBetaFilter     = false; // position-velocity smoothing in XY
    bool enableAdaptiveNoise       = false; // adapt Alpha/Beta by HDOP
    bool enableOutlierGating       = false; // reject large innovations
    bool enableZUPT                = false; // Zero-Velocity Update when standing

    // Thresholds / parameters (weiter für Telemetrie / Status genutzt)
    double   maxHdop        = 1.5;   // akzeptabler Bereich (für Status)
    uint8_t  minSatellites  = 7;
    uint32_t maxFixAgeMs    = 1500;

    // GOOD/WARN-gates for fix status
    double hdopGood  = 0.9; // HDOP <= hdopGood    → GOOD
    double hdopWarn  = 1.5; // HDOP <= hdopWarn    → WARN, other BAD

    uint8_t satsGood = 9;   // sats >= satsGood    → GOOD
    uint8_t satsWarn = 7;   // sats >= satsWarn    → WARN, other BAD

    double zuptSpeedThreshMps = 0.05; 

    // Alpha-Beta baseline
    double alpha_base = 0.40, beta_base = 0.05;
    double alpha_good = 0.60, beta_good = 0.10;
    double alpha_poor = 0.20, beta_poor = 0.02;

    // Outlier gating
    double gateMetersBase   = 5.0;
    double gateMetersPerMps = 2.0;
    double gateMetersMax    = 12.0;

    // --- simpler filters  --------------------------------------
    bool    enableMedianFilter    = false; // Sliding-Median
    bool    enableMeanFilter      = false; // Sliding-Mean (Moving Average)
    uint8_t medianWindowSize      = 5;     // window [1..MAX]
    uint8_t meanWindowSize        = 5;     // window [1..MAX]
  };

  // Optional wrapper
  struct CampusFrame {
    double originLat = 0.0;
    double originLon = 0.0;
    double cornerXLat = 0.0;
    double cornerXLon = 0.0;
    double cornerYLat = 0.0;
    double cornerYLon = 0.0;
  };

  // Telemetrie-Container
  struct Telemetry {
    // Raw reads
    bool     haveXY  = false;
    float    x_m     = 0.0f;
    float    y_m     = 0.0f;

    bool     llValid = false;
    double   latDeg  = 0.0;
    double   lonDeg  = 0.0;

    bool     qualOk  = false;
    double   hdop    = 0.0;
    int      sats    = 0;
    uint32_t ageMs   = 0;

    // Derived
    bool     freshFix = false;

    // outputs
    double X_out   = 0.0;
    double Y_out   = 0.0;
    double LAT_out = 0.0;
    double LON_out = 0.0;
    float  HDOP_out = 0.0f;
    float  Sats_out = 0.0f;
    float  Age_out  = 0.0f;
    int    Fix_out  = 0;

    // Status
    uint8_t fixStatusCode = 0;
    char    debug[160]    = {0};
  };

  
  enum class OriginSetStatus : uint8_t {
    SET_OK = 0,
    NO_FRESH_FIX,
    FRAME_NOT_READY
  };

  GpsManager() = default;

  // Initialize UART, TinyGPS++, and optionally the campus frame and filters.
  bool begin(HardwareSerial& port, const Config& cfg);

  // Convenience: Default-config for THD/Campus-Frame
  static CampusFrame thdCampusFrame();
  static void        thdTouchUpOrigin(double& latDeg, double& lonDeg);
  static Config      makeThdCampusDefaultConfig();
  bool               begin();
  bool               setTouchUpOriginThdDefault(bool resetFilterAfter = false);

  // Pump NMEA bytes from UART into TinyGPS++; processes new fixes internally.
  void poll();

  // True if TinyGPS++ currently holds a valid location. (non-const due to TinyGPS++ API)
  bool hasValidLocation();

  // Retrieve current latitude/longitude if valid. (raw GNSS, not filtered)
  bool getCurrentLatLon(double& latDeg, double& lonDeg);

  // Convenience wrapper that currently forwards to toCampusXY() and returns
  // campus-aligned XY coordinates using the configured frame.
  bool latLonToLocalXY(double latDeg, double lonDeg,
                       float& x_m, float& y_m) const;

  // Convert any lat/lon to campus XY [m]; false if frame not configured.
  bool toCampusXY(double latDeg, double lonDeg, float& x_m, float& y_m) const;

  // Return the latest campus XY [m]. Wenn Filter aktiv, dann geglättete XY.
  // False if no fix or frame not configured. (non-const due to TinyGPS++ API)
  bool getCampusXY(float& x_m, float& y_m);

  // set new origin. only call after begin
  bool setTouchUpOriginLatLon(double latDeg, double lonDeg);

  // Extents along the orthonormal campus axes (informational).
  double getCampusXmax() const { return _campusAxesReady ? _xmax_m : 0.0; }
  double getCampusYmax() const { return _campusAxesReady ? _ymax_m : 0.0; }

  // Optional helpers for debugging/telemetry (do not change main interface).
  bool getLastQuality(double& hdop, int& sats, uint32_t& ageMs) const;
  bool getLastCogDeg(double& cogDeg) const;
  bool getLastSogMps(double& sogMps) const;
  void resetFilter();

  // --- Telemetrie & Origin-Set --------------------------------
  void            buildTelemetry(Telemetry& t, uint32_t freshMaxMs = 0, float sentinel = 0.0f);
  OriginSetStatus trySetTouchUpOriginFromCurrentFix(uint32_t freshMaxMs = 0, bool resetFilterAfter = true);
  void            printTelemetry(Stream& out, const Telemetry& t,
                                uint8_t xyPrec = 2, uint8_t hdopPrec = 2, uint8_t llaPrec = 7) const;
  void            printTelemetry(Stream& out, uint32_t freshMaxMs = 0, float sentinel = 0.0f);

  // --- GNSS fix quality status (for FSM / HMI) -------------------------------
  enum class FixStatus : uint8_t {
    NO_DATA = 0,  // no valid fix processed yet
    BAD,          // inside age window, but fails HDOP / sats thresholds
    WARN,         // usable but degraded
    GOOD          // good quality fix
  };

  FixStatus   getFixStatus() const;
  const char* getFixStatusString(FixStatus s) const;
  const char* getFixStatusString() const { return getFixStatusString(getFixStatus()); }
  void buildDebugString(char* buf, size_t len) const;

  // --- Local time helpers (UTC -> CET/CEST)
  uint16_t getYearCET()  const;
  uint8_t  getMonthCET() const;
  uint8_t  getDayCET()   const;
  uint8_t  getHourCET()  const;
  uint8_t  getMinuteCET()const;
  uint8_t  getSecondCET()const;

private:
  // --- UART / GPS ---
  HardwareSerial* _serial = nullptr;
  TinyGPSPlus     _gps;

  // --- Campus frame internals (immutable after begin) ---
  bool   _campusFrameEnabled = false;
  bool   _campusAxesReady    = false;
  // Offset in Campus-XY-System (in meter)
  double _originOffsetX_m = 0.0;
  double _originOffsetY_m = 0.0;

  double _campusLat0 = 0.0, _campusLon0 = 0.0; // origin (deg)
  double _kx_m_per_deg = 0.0;                  // meters per degree of longitude at origin latitude
  double _ky_m_per_deg = 0.0;                  // meters per degree of latitude

  // Orthonormal basis vectors for campus frame expressed in EN (east,north)
  double _exx = 1.0, _exy = 0.0;  // +X axis components
  double _eyx = 0.0, _eyy = 1.0;  // +Y axis components

  // Extents along axes (meters)
  double _xmax_m = 0.0, _ymax_m = 0.0;

  // --- Filtering state ---
  static constexpr uint8_t MAX_FILTER_WINDOW = 25;

  Config   _cfg{};
  bool     _filterActive = false;   
  bool     _stateValid   = false;   
  double   _x = 0.0, _y = 0.0;      
  double   _vx = 0.0, _vy = 0.0;    
  uint32_t _lastUpdateMs = 0;

 
  double  _histX[MAX_FILTER_WINDOW];
  double  _histY[MAX_FILTER_WINDOW];
  uint8_t _histCount = 0;           

 
  double   _lastMeasX = 0.0, _lastMeasY = 0.0;
  double   _lastHdop  = 99.0;
  int      _lastSats  = 0;
  uint32_t _lastAgeMs = 0;
  double   _lastCogDeg = NAN;
  double   _lastSogMps = 0.0;

  // Campus frame
  void _initCampusFrame(double lat0, double lon0,
                        double cxLat, double cxLon,
                        double cyLat, double cyLon);

  // Helpers
  void   _toEN(double latDeg, double lonDeg, double& E_m, double& N_m) const;
  void   _ingestIfUpdated();       // process a new TinyGPS++ fix if available

  // old quality gatin, 
  bool   _qualityPass(double hdop, int sats, uint32_t ageMs) const;

  // Latenzkompensation for later optional usage
  void   _applyLatencyComp(double& E_m, double& N_m,
                           double sog_mps, double cog_deg, uint32_t ageMs) const;
  void   _filterUpdate(double zX, double zY, double dt, double hdop, double sog_mps);

  // simpler filters
  void   _pushSample(double x, double y);
  void   _computeMean(double& fx, double& fy) const;
  void   _computeMedian(double& fx, double& fy) const;

  // Time conversion helper: UTC -> CET/CEST (Europe/Berlin rules)
  bool   _utcToCet(int& y, int& m, int& d, int& h, int& mi, int& s) const;
  void   _configureNeo6m5Hz();
};
