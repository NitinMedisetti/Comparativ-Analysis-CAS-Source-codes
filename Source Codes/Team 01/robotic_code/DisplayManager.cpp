// the goal of this DisplayManager was to keep the lines and complexity in the .ino file as few as possible. 

#include "DisplayManager.h"

// define size in pexels
static const uint8_t OLED_W = 128;
static const uint8_t OLED_H = 32;

static const uint8_t OLED_ADDR = 0x3C;

// ESP32 I2C Pins
static const int8_t I2C_SDA = 21;
static const int8_t I2C_SCL = 22;

DisplayManager::DisplayManager()
  : _disp(OLED_W, OLED_H, &Wire, -1) {}

bool DisplayManager::begin() {
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!_disp.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    _ready = false;
    return false;
  }

  _ready = true;

  _disp.clearDisplay();
  _disp.setTextColor(SSD1306_WHITE);
  _disp.setTextSize(_textSize);
  _disp.setCursor(0, 0);
  _disp.display();
  return true;
}

void DisplayManager::clear() {
  if (!_ready) return;
  _disp.clearDisplay();
  _disp.display();
}

bool DisplayManager::isNonEmpty(const char* s) const {
  return (s != nullptr && s[0] != '\0');
}

void DisplayManager::show(const char* line1, const char* line2, const char* line3) {
  if (!_ready) return;

  _disp.clearDisplay();
  _disp.setTextColor(SSD1306_WHITE);
  _disp.setTextSize(_textSize);

  uint8_t idx = 0;

  if (isNonEmpty(line1)) { printLine(idx++, line1); }
  if (idx < 3 && isNonEmpty(line2)) { printLine(idx++, line2); }
  if (idx < 3 && isNonEmpty(line3)) { printLine(idx++, line3); }

  _disp.display();
}

void DisplayManager::printLine(uint8_t lineIndex, const char* text) {
  const uint8_t lineHeight = 8 * _textSize;                 
  const uint8_t step = lineHeight + _lineGapPx;             // add the line gap
  const uint8_t y = lineIndex * step;

  if (y >= OLED_H) return;

  char buf[MaxCharsPerLine + 1];
  uint8_t i = 0;
  for (; i < MaxCharsPerLine && text[i] != '\0'; ++i) {
    buf[i] = text[i];
  }
  buf[i] = '\0';

  _disp.setCursor(0, y);
  _disp.print(buf);
}
