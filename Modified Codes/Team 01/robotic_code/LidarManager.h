#pragma once
#include <Arduino.h>

#define LIDAR_POINT_PER_PACK 12
#define LIDAR_HEADER 0x54
#define LIDAR_MAX_ANGLES 360

class LidarManager {
public:
  struct Config {
    int rxPin = 27;              // ESP32 RX2  <- LD20 TX
    int txPin = 33;              // ESP32 TX2  <- LD20 RX (optional)
    uint32_t baudRate = 230400;  // LD20 default
    int ledPin = -1;
    uint16_t maxDistance = 12000; // mm
    uint8_t streamInterval = 15;
  };

  struct ScanData {
    int distanceMm[LIDAR_MAX_ANGLES];
    uint32_t packetsReceived;
    uint32_t crcErrors;
    bool dataReady;
  };

  typedef struct __attribute__((packed)) {
    uint16_t distance;
    uint8_t intensity;
  } LidarPoint;

  typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPoint point[LIDAR_POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
  } LidarFrame;

  bool begin(HardwareSerial& port, const Config& cfg);
  void update();
  ScanData getScanData() const;
  float getCrcErrorRate() const;
  void clearData();

private:
  static const uint8_t _crcTable[256];
  uint8_t calcCRC8(uint8_t* p, uint8_t len);
  void parsePacket(LidarFrame* frame);
  void receiveData();

private:
  HardwareSerial* _port = nullptr;
  Config _cfg;

  int _distanceMm[LIDAR_MAX_ANGLES] = {0};
  uint32_t _packetsReceived = 0;
  uint32_t _crcErrors = 0;

  uint8_t _rxBuffer[sizeof(LidarFrame)];
  uint8_t _rxIndex = 0;
  bool _inPacket = false;
};
