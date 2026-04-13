#pragma once
#include <Arduino.h>

class UltrasonicSensor {
  public:
  UltrasonicSensor(uint8_t triggerPin, uint8_t echoPin) 
  : m_triggerPin(triggerPin), m_echoPin(echoPin) 
  {}

  void begin() {
    pinMode(m_triggerPin, OUTPUT);
    pinMode(m_echoPin, INPUT);
  }

  uint16_t readDistance()
  {
    digitalWrite(m_triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(m_triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(m_triggerPin, LOW);

    uint32_t duration = pulseIn(m_echoPin, HIGH, 30000); 
    return static_cast<uint16_t>((duration * 0.034f) / 2.0f);
  }

  private:
  const uint8_t m_triggerPin;
  const uint8_t m_echoPin;
};