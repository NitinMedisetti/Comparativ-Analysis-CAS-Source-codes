#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <esp_now.h>

// OLED config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define LED_PIN 2

// Create display object (included once in your sketch via header)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Global state (defined here so including main gets them)
// NOTE: these are actual definitions (not extern) — keep header included only once.
String oledStatusMessage = "INITIALIZING...";
unsigned long oledDisplayTimeout = 0;

// External globals that must be defined in the main sketch
extern float currentHeading; // Heading from BNO055
extern uint8_t sys_cal;      // System Calibration Status
extern float currentLat;     // Latitude from GPS (float, not String)

// =======================================
// ARROW DISPLAY FUNCTIONS (128x32 OLED)
// =======================================

// Clears screen and displays for 3s
void showArrowScreen(void (*drawFunc)()) {
    display.clearDisplay();
    drawFunc();
    display.display();
    oledDisplayTimeout = millis() + 3000;
}

// ---- FORWARD ARROW ----
void drawAlignedScreen() {
    display.setTextSize(2);
    display.setCursor(20, 8);
    display.println("ALIGNED");
}
void showAlignedScreen() { showArrowScreen(drawAlignedScreen); }

void drawForwardArrow() {
    int mid = SCREEN_WIDTH / 2;

    // Arrow triangle (tip)
    display.fillTriangle(mid, 0, mid - 10, 12, mid + 10, 12, SSD1306_WHITE);
    // Arrow shaft
    display.fillRect(mid - 3, 12, 6, 20, SSD1306_WHITE);
}
void showForwardArrow() { showArrowScreen(drawForwardArrow); }

// ---- BACKWARD ARROW ----
void drawBackwardArrow() {
    int mid = SCREEN_WIDTH / 2;

    // Triangle pointing down
    display.fillTriangle(mid, 31, mid - 10, 20, mid + 10, 20, SSD1306_WHITE);
    // Shaft
    display.fillRect(mid - 3, 0, 6, 20, SSD1306_WHITE);
}
void showBackwardArrow() { showArrowScreen(drawBackwardArrow); }

// ---- LEFT ARROW ----
void drawLeftArrow() {
    int midH = SCREEN_HEIGHT / 2;

    display.fillTriangle(0, midH, 12, midH - 10, 12, midH + 10, SSD1306_WHITE);
    display.fillRect(12, midH - 3, 40, 6, SSD1306_WHITE);
}
void showLeftArrow() { showArrowScreen(drawLeftArrow); }

// ---- RIGHT ARROW ----
void drawRightArrow() {
    int midH = SCREEN_HEIGHT / 2;

    display.fillTriangle(127, midH, 115, midH - 10, 115, midH + 10, SSD1306_WHITE);
    display.fillRect(75, midH - 3, 40, 6, SSD1306_WHITE);
}
void showRightArrow() { showArrowScreen(drawRightArrow); }

// ---- ROTATE LEFT (circular arrow) ----
void drawRotateLeftArrow() {
    display.setTextSize(2);
    display.setCursor(40, 8);
    display.println("ROT_L"); // Arrowhead
}
void showRotateLeftArrow() { showArrowScreen(drawRotateLeftArrow); }

// ---- ROTATE RIGHT (circular arrow) ----
void drawRotateRightArrow() {
    display.setTextSize(2);
    display.setCursor(40, 8);
    display.println("ROT_R"); // Arrowhead
}
void showRotateRightArrow() { showArrowScreen(drawRotateRightArrow); }

// ---- STOP SCREEN ----
void drawStopScreen() {
    display.setTextSize(2);
    display.setCursor(40, 8);
    display.println("STOP");
}
void showStopScreen() { showArrowScreen(drawStopScreen); }

inline void displayCommandOnOLED(const char* cmd) {
  if      (strcmp(cmd,"FWD")==0)  showForwardArrow();
  else if (strcmp(cmd,"BACK")==0) showBackwardArrow();
  else if (strcmp(cmd,"LEFT")==0) showLeftArrow();
  else if (strcmp(cmd,"RIGHT")==0)showRightArrow();
  else if (strcmp(cmd,"ROT_L")==0)showRotateLeftArrow();
  else if (strcmp(cmd,"ROT_R")==0)showRotateRightArrow();
  else if (strcmp(cmd,"STOP")==0) showStopScreen();
  else if (strcmp(cmd,"ALIGNED")==0) showAlignedScreen();
}



// Helper LED blink
void blinkLED(int times) {
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// Initialize OLED and show "TEAM 3" splash for 10s
bool initOLED() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        return false;
    }

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, (SCREEN_HEIGHT - 16) / 2);
    display.println("TEAM 3");
    display.display();

    oledStatusMessage = "TEAM 3";
    oledDisplayTimeout = millis() + 10000; // 10s splash

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    return true;
}

void updateOLED() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // show splash / temporary status
  if (millis() < oledDisplayTimeout) {
    display.setFont(NULL);
    display.setTextSize(2);
    display.setCursor(0, (SCREEN_HEIGHT - 16) / 2);
    if (oledStatusMessage.length() > 16) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println(oledStatusMessage);
    } else {
      display.println(oledStatusMessage);
    }
    display.display();
    return;
  }

  // Default to SEARCHING... after splash/timeout
  if (oledStatusMessage.equals("TEAM 3") || oledStatusMessage.equals("INITIALIZING...")) {
      oledStatusMessage = "SEARCHING...";
  } else if (oledStatusMessage.equals("STOPPING...") || oledStatusMessage.startsWith("HEADING TO")) {
      oledStatusMessage = "SEARCHING...";
  } else if (!oledStatusMessage.equals("SEARCHING...")) {
      oledStatusMessage = "SEARCHING...";
  }

  // Standard live display
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("STATUS: " + oledStatusMessage);

  // heading & calibration
  display.setCursor(0, 10);
  char line2Buffer[17];
  snprintf(line2Buffer, sizeof(line2Buffer), "head:%.1f S:%d", currentHeading, sys_cal);
  display.println(line2Buffer);

  // GPS lat
  display.setCursor(0, 20);
  if (currentLat != 0.0f) {
      char line3Buffer[17];
      snprintf(line3Buffer, sizeof(line3Buffer), "LAT:%.3f", currentLat);
      display.println(line3Buffer);
  } else {
      display.println("GPS: 0 sat");
  }

  display.display();
}

// ESP-NOW receiver callback matching modern esp32-core signature
typedef struct espMessage_t {
  char cmd[20];
} espMessage_t;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  espMessage_t msg;
  if (len >= (int)sizeof(espMessage_t)) {
     memcpy(&msg, data, sizeof(espMessage_t));

    if (strcmp(msg.cmd, "STOP") == 0) {
        blinkLED(1);
        oledStatusMessage = "STOPPING...";
        oledDisplayTimeout = millis() + 3000;
    } else if (strcmp(msg.cmd, "POST BUILDING") == 0) {
        blinkLED(2);
        oledStatusMessage = "HEADING TO POST BLD";
        oledDisplayTimeout = millis() + 3000;
    } else if (strcmp(msg.cmd, "H BUILDING") == 0) {
        blinkLED(3);
        oledStatusMessage = "HEADING TO H BLD";
        oledDisplayTimeout = millis() + 3000;
    }
  }
}

#endif // OLED_DISPLAY_H
