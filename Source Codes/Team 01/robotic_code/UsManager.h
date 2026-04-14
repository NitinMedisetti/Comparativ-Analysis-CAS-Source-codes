#ifndef US_MANAGER_H
#define US_MANAGER_H

#include <Arduino.h>

// ESP32 / Arduino: FreeRTOS is available
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class UsManager {
public:
  struct Pins {
    int trigFront;
    int echoFront;

    int trigLeft;
    int echoLeft;

    int trigRight;
    int echoRight;

    int trigBack;
    int echoBack;

    // Front angled (45°)
    int trigFrontAngled;
    int echoFrontAngled;

    Pins()
    : trigFront(32), echoFront(25),
      trigLeft(4),  echoLeft(18),
      trigRight(13), echoRight(19),
      trigBack(5),  echoBack(14),
      trigFrontAngled(23), echoFrontAngled(26) {}
  };

  UsManager();                    // Default-Pins
  explicit UsManager(const Pins& pins);

  void begin();

  // Optional: call in loop() – measurements run in background task.
  void update();

  // Latest cached values (cm). "No echo" returns maxDistanceCm().
  int readFrontCm() const;
  int readLeftCm() const;
  int readRightCm() const;
  int readBackCm() const;
  int readFrontAngledCm() const;

  void setTimeoutUs(uint32_t timeoutUs) { timeoutUs_ = timeoutUs; }
  void setMaxDistanceCm(int maxDistanceCm) { maxDistanceCm_ = maxDistanceCm; }

  // Delays to reduce cross-talk and control refresh rate.
  void setInterSensorDelayMs(uint32_t ms) { interSensorDelayMs_ = ms; }
  void setSweepDelayMs(uint32_t ms) { sweepDelayMs_ = ms; }

  int maxDistanceCm() const { return maxDistanceCm_; }

private:
  int readCmRaw_(int trigPin, int echoPin, uint32_t timeoutUs, int maxDistanceCm);

  static void taskThunk_(void* param);
  void taskLoop_();

  Pins pins_;
  uint32_t timeoutUs_ = 25000; // 25 ms
  int maxDistanceCm_ = 250;    // cm

  uint32_t interSensorDelayMs_ = 5;
  uint32_t sweepDelayMs_ = 0;

  // Cached values (written by task, read by loop)
  volatile int lastFrontCm_ = 250;
  volatile int lastLeftCm_ = 250;
  volatile int lastRightCm_ = 250;
  volatile int lastBackCm_ = 250;
  volatile int lastFrontAngledCm_ = 250;

  TaskHandle_t taskHandle_ = nullptr;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};

#endif // US_MANAGER_H
