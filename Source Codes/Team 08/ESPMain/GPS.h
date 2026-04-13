#pragma once
#include <TinyGPS++.h>
#include <Arduino.h>

class GPSModule
{
public:
  GPSModule(HardwareSerial &serialPort, uint32_t baud = 9600)
      : m_gpsSerial(serialPort), baudRate(baud) {}

  void begin(uint8_t rxPin, uint8_t txPin)
  {
    m_gpsSerial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
  }

  void update()
  {
    while (m_gpsSerial.available())
    {
      gps.encode(m_gpsSerial.read());
    }
  }

  bool locationValid()
  {
    return gps.location.isValid();
  }

  double getLatitude()
  {
    return gps.location.isValid() ? gps.location.lat() : 0.0;
  }

  double getLongitude()
  {
    return gps.location.isValid() ? gps.location.lng() : 0.0;
  }

  // ===== NEW FUNCTIONS FOR GPS QUALITY =====

  // Get number of satellites
  int getSatellites()
  {
    return gps.satellites.isValid() ? gps.satellites.value() : 0;
  }

  // Get HDOP (Horizontal Dilution of Precision)
  // Lower is better: <1 = Ideal, 1-2 = Excellent, 2-5 = Good, 5-10 = Moderate, 10-20 = Fair, >20 = Poor
  float getHDOP()
  {
    return gps.hdop.isValid() ? (gps.hdop.value() / 100.0f) : 99.99f;
  }

  // Get altitude (optional, useful for debugging)
  double getAltitude()
  {
    return gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  }

  // Get speed (optional, useful for velocity estimation)
  double getSpeedMps()
  {
    return gps.speed.isValid() ? gps.speed.mps() : 0.0;
  }

  // Get course/heading from GPS (optional)
  double getCourse()
  {
    return gps.course.isValid() ? gps.course.deg() : 0.0;
  }

  // Check if GPS has a fix
  bool hasFix()
  {
    return gps.location.isValid() && gps.satellites.value() >= 4;
  }

  // Get age of last GPS update (in milliseconds)
  uint32_t getLocationAge()
  {
    return gps.location.age();
  }

  // Check if data is stale (no update for X milliseconds)
  bool isDataStale(uint32_t maxAge_ms = 2000)
  {
    return gps.location.age() > maxAge_ms;
  }

private:
  HardwareSerial &m_gpsSerial;
  uint32_t baudRate;
  TinyGPSPlus gps;
};