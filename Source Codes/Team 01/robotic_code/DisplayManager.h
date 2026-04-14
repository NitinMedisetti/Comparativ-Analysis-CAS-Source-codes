#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class DisplayManager {
public:
  DisplayManager();

  bool begin();
  void clear();


  void show(const char* line1, const char* line2 = nullptr, const char* line3 = nullptr);

private:
  static const uint8_t MaxCharsPerLine = 20;

  void printLine(uint8_t lineIndex, const char* text);
  bool isNonEmpty(const char* s) const;

private:
  Adafruit_SSD1306 _disp;
  bool _ready = false;

  // text size can be changed
  const uint8_t _textSize = 1;

  // distance in pixels between lines
  const uint8_t _lineGapPx = 4;
};
