#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <LittleFS.h>

// ===== AP configuration ======================================================
#define WEBHMI_WIFI_SSID   "ESP32HMI2"
#define WEBHMI_WIFI_PASS   "JonasESP"
#define WEBHMI_AP_IP       192,168,4,1
#define WEBHMI_AP_SUBNET   255,255,255,0
// ============================================================================

// ===== Logging configuration ================================================
#define WEBHMI_LOG_PATH "/log.csv"
// ============================================================================

class WebHMI {
public:
  static const uint8_t MAX_ENTRIES = 20;
  static const uint8_t MAX_BUTTONS = 20;

  // Operator UI: Buttons (defined in setup, order is preserved)
  static const uint8_t MAX_OP_PRIMARY   = 8;
  static const uint8_t MAX_OP_SECONDARY = 20;

  // Dev Mode: Numeric input fields
  static const uint8_t MAX_DEV_INPUTS   = 8;

  WebHMI();

  void begin();
  void setTitle(const char* title);

  // Dev/legacy values (compatible, incl. strings)
  void showValue(const char* name, double value);
  void showValue(const char* name, const char* value);
  void showValue(const char* name, const String& value);

  // Dev/legacy buttons
  // - Creates the button if it does not exist yet, using initialState.
  // - Does NOT overwrite the state if the button already exists (web toggles remain intact).
  void showButton(const char* name, bool initialState);

  // Dev/legacy button state in code
  bool getButtonState(const char* name) const;
  void setButtonState(const char* name, bool state);

  // Operator UI: on/off (when off => operator section is hidden)
  void enableOperatorUI(bool on);

  // Operator UI: define buttons in setup (order = render order)
  void opAddPrimaryButton(const char* cmdId, const char* label);
  void opAddSecondaryButton(const char* cmdId, const char* label);

  // Operator UI: text fields like showValue, but semantically "Operator"
  void opShowText(const char* key, const char* text);
  void opShowText(const char* key, const String& text);

  // Operator commands: read button-press events from the web UI
  bool opCommandAvailable() const;
  bool opPopCommand(char* outCmd, size_t outCmdLen);

  // Demo
  void enableDemo(bool on);

  // Dev-only: loop cycle stats (min/max/avg)
  void enableCycleStats(bool on);

  // Dev-only: Numeric input field (float)
  // - Define in setup (one line); appears in the Dev panel as label + input field.
  // - Letters are rejected server-side (allowed: 0-9, +/- , '.' ','; at least 1 digit required).
  void showDevInputFloat(const char* name, float defaultValue);

  // Dev-only: read value (for dev inputs)
  float getValue(const char* name, float fallback = 0.0f) const;

  void tick(); // call from loop()

private:
  struct Entry {
    char  name[32];
    bool  inUse;
    bool  isString;
    double value;
    char  text[96];
  };

  struct Button {
    char name[32];
    bool inUse;
    bool state;
  };

  struct OpButtonDef {
    char cmd[32];
    char label[32];
    bool inUse;
  };

  struct DevInput {
    char  name[32];
    bool  inUse;
    float value;
  };

  Entry     _entries[MAX_ENTRIES];
  Button    _buttons[MAX_BUTTONS];
  OpButtonDef _opPrimary[MAX_OP_PRIMARY];
  OpButtonDef _opSecondary[MAX_OP_SECONDARY];
  DevInput  _devInputs[MAX_DEV_INPUTS];

  AsyncWebServer* _server;
  String _title;

  // Operator
  bool _operatorEnabled;

  // Operator command event (single-slot, sequenced)
  volatile uint32_t _opCmdSeq;
  uint32_t _opCmdConsumedSeq;
  char _opLastCmd[32];

  // Demo
  bool _demoOn;
  uint32_t _demoMs;
  unsigned long _t0;
  int _cntA;
  float _cntB;

  // Logging
  bool _isLogging;
  bool _headerWritten;
  File _logFile;
  unsigned long _logStartMs;
  unsigned long _lastLogMs;
  static constexpr unsigned long kLogIntervalMs = 200UL;

  // Cycle stats (dev-only view)
  bool _cycleStatsEnabled;
  uint32_t _cycleLastUs;
  uint32_t _cycleWinStartMs;
  uint32_t _cycleMinUs;
  uint32_t _cycleMaxUs;
  uint64_t _cycleSumUs;
  uint32_t _cycleCount;
  float _cycleMinMs;
  float _cycleMaxMs;
  float _cycleAvgMs;

  // Logging helpers
  void _startLogging();
  void _stopLogging(bool fromAuto=false);
  void _writeHeader();
  void _writeRow();
  void _flushIfOpen();

  // Webserver
  void _setupRoutes();
  int  _findIndex(const char* name);
  int  _firstFree();

  int  _findButtonIndex(const char* name) const;
  int  _firstFreeButton() const;

  int  _firstFreeOpPrimary() const;
  int  _firstFreeOpSecondary() const;

  int  _findDevInputIndex(const char* name) const;
  int  _firstFreeDevInput() const;

  String _buildJson() const;
  String _buildHtml() const;

  void _demoStep();
  void _cycleStatsTick();
};
