#ifndef GPS_MODULE_H
#define GPS_MODULE_H


// Set these after selecting your origin point
#define LAT_ORIGIN  48.829876
#define LON_ORIGIN  12.954973

typedef struct {
  TinyGPSPlus gps;
  HardwareSerial* SerialGPS;

  double latitude;
  double longitude;
  int satellites;
  double altitude;
  double local_x;  // East  (meters)
  double local_y;  // North (meters)

} GPSModule;

// --- INTERNAL: convert lat/lon to meters relative to origin ---
static void convertToLocal(double lat, double lon, double* x, double* y) {
  const double a = 6378137.0;
  const double b = 6356752.3142;
  const double e2 = (a*a - b*b) / (a*a);

  double dLat = lat - LAT_ORIGIN;
  double dLon = lon - LON_ORIGIN;

  double latRad = LAT_ORIGIN * M_PI / 180.0;

  double metersPerDegLat = (M_PI / 180.0) * a * (1 - e2) / pow(1 - e2 * sin(latRad) * sin(latRad), 1.5);
  double metersPerDegLon = (M_PI / 180.0) * a * cos(latRad) / sqrt(1 - e2 * sin(latRad) * sin(latRad));

  *y = dLat * metersPerDegLat;  // North
  *x = dLon * metersPerDegLon;  // East
}

// ===================== UBX FUNCTIONS (MUST BE ABOVE GPS_init) =====================

// checksum
static void ubxChecksum(const uint8_t *payload, uint16_t len, uint8_t &ckA, uint8_t &ckB) {////////////////////////
  ckA = 0; ckB = 0;////////////////////////
  for (uint16_t i = 0; i < len; i++) {////////////////////////
    ckA = ckA + payload[i];////////////////////////
    ckB = ckB + ckA;////////////////////////
  }////////////////////////
}////////////////////////
////////////////////////
// send UBX////////////////////////
static void sendUBX(HardwareSerial &ser, const uint8_t *msg, uint16_t len) {////////////////////////
  ser.write(0xB5);////////////////////////
  ser.write(0x62);////////////////////////
  ser.write(msg, len);////////////////////////
////////////////////////
  uint8_t ckA, ckB;////////////////////////
  ubxChecksum(msg, len, ckA, ckB);////////////////////////
  ser.write(ckA);////////////////////////
  ser.write(ckB);////////////////////////
  ser.flush();////////////////////////
}////////////////////////

// set update rate Hz (NEO-6M: 5Hz usually OK)
static void GPS_setUpdateRateHz(HardwareSerial &ser, uint8_t hz) {
  if (hz < 1) hz = 1;
  uint16_t measRateMs = (uint16_t)(1000 / hz);

  uint8_t msg[] = {
    0x06, 0x08, 0x06, 0x00,
    (uint8_t)(measRateMs & 0xFF), (uint8_t)(measRateMs >> 8),
    0x01, 0x00,   // navRate = 1
    0x01, 0x00    // timeRef = 1 (GPS time)
  };

  sendUBX(ser, msg, sizeof(msg));
}

// optional save (may not persist on some clones)
static void GPS_saveConfig(HardwareSerial &ser) {
  uint8_t msg[] = {
    0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00,  // clearMask
    0xFF, 0xFF, 0x00, 0x00,  // saveMask
    0x00, 0x00, 0x00, 0x00,  // loadMask
    0x00
  };
  sendUBX(ser, msg, sizeof(msg));
}

// ===================== GPS API =====================

static void GPS_init(GPSModule* module, int rxPin, int txPin) {
  module->SerialGPS = &Serial1;

  module->latitude  = 0;
  module->longitude = 0;
  module->satellites = 0;
  module->altitude = 0;
  module->local_x = 0;
  module->local_y = 0;

  module->SerialGPS->begin(9600, SERIAL_8N1, rxPin, txPin);

  delay(200); // let GPS boot           ///////////////////////
  GPS_setUpdateRateHz(*module->SerialGPS, 5); // set 5 Hz           ///////////////////////
  delay(50);            ///////////////////////

  // GPS_saveConfig(*module->SerialGPS); // optional
  Serial.println("[GPS] Requested 5 Hz update rate (UBX-CFG-RATE).");//
}

static void GPS_getReadings(GPSModule* module) {
  int maxBytes = 256;

  while (module->SerialGPS->available() > 0 && maxBytes--) {
    module->gps.encode(module->SerialGPS->read());

    if (module->gps.location.isUpdated() && module->gps.location.isValid()) {
      module->latitude   = module->gps.location.lat();
      module->longitude  = module->gps.location.lng();

      if (module->gps.satellites.isValid())
        module->satellites = module->gps.satellites.value();

      if (module->gps.altitude.isValid())
        module->altitude = module->gps.altitude.meters();

      convertToLocal(module->latitude, module->longitude,
                     &module->local_x, &module->local_y);
    }
  }
}

static void GPS_returnReadings(GPSModule* module,
                        double* Latitude, double* Longitude,
                        int* Satellites, double* Altitude,
                        double* X_local, double* Y_local)
{
  *Latitude = module->latitude;
  *Longitude = module->longitude;
  *Satellites = module->satellites;
  *Altitude = module->altitude;
  *X_local = module->local_x;
  *Y_local = module->local_y;
}

#endif
