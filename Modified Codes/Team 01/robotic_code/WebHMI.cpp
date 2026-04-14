#include "WebHMI.h"
#include <cstring>

// ----------------------------- JSON Escape ----------------------------------
static inline void jsonAppendEscaped(String& out, const char* s) {
  for (const char* p = s; *p; ++p) {
    const char c = *p;
    switch (c) {
      case '\\': out += F("\\\\"); break;
      case '\"': out += F("\\\""); break;
      case '\n': out += F("\\n");  break;
      case '\r': out += F("\\r");  break;
      case '\t': out += F("\\t");  break;
      default:
        if ((uint8_t)c < 0x20) {
          // skip control chars
        } else {
          out += c;
        }
        break;
    }
  }
}

// ---- Helper: print number with comma as decimal separator in CSV ----------
static inline void printWithComma(Print& out, double value, uint8_t decimals) {
  char buf[32];
  dtostrf(value, 0, (unsigned char)decimals, buf);
  for (char* p = buf; *p; ++p) {
    if (*p == '.') *p = ',';
  }
  out.print(buf);
}

// ---- Helper: safely print CSV string (delimiter ';' => quotes) ------------
static inline void printCsvEscaped(Print& out, const char* s) {
  if (!s) { out.print(""); return; }

  bool needQuotes = false;
  for (const char* p = s; *p; ++p) {
    if (*p == ';' || *p == '\"' || *p == '\n' || *p == '\r') { needQuotes = true; break; }
  }
  if (!needQuotes) { out.print(s); return; }

  out.print('\"');
  for (const char* p = s; *p; ++p) {
    if (*p == '\"') out.print("\"\"");
    else out.print(*p);
  }
  out.print('\"');
}

static inline bool isAllowedFloatChar(char c) {
  return (c >= '0' && c <= '9') || c == '.' || c == ',' || c == '-' || c == '+';
}

static inline bool isValidFloatString(const String& s) {
  if (s.length() == 0) return false;
  bool hasDigit = false;
  for (size_t i = 0; i < s.length(); ++i) {
    const char c = s[i];
    if (!isAllowedFloatChar(c)) return false;
    if (c >= '0' && c <= '9') hasDigit = true;
  }
  return hasDigit;
}

static inline float parseFloatCommaDot(String s) {
  s.replace(',', '.');
  return s.toFloat();
}

// ----------------------------------------------------------------------------

WebHMI::WebHMI()
: _server(nullptr),
  _title("WebHMI"),
  _operatorEnabled(false),
  _opCmdSeq(0),
  _opCmdConsumedSeq(0),
  _demoOn(false),
  _demoMs(0),
  _t0(0),
  _cntA(0),
  _cntB(0.0f),
  _isLogging(false),
  _headerWritten(false),
  _logStartMs(0),
  _lastLogMs(0),
  _cycleStatsEnabled(false),
  _cycleLastUs(0),
  _cycleWinStartMs(0),
  _cycleMinUs(0),
  _cycleMaxUs(0),
  _cycleSumUs(0),
  _cycleCount(0),
  _cycleMinMs(0.0f),
  _cycleMaxMs(0.0f),
  _cycleAvgMs(0.0f)
{
  for (int i = 0; i < MAX_ENTRIES; ++i) {
    _entries[i].inUse = false;
    _entries[i].isString = false;
    _entries[i].value = 0.0;
    _entries[i].name[0] = '\0';
    _entries[i].text[0] = '\0';
  }

  for (int i = 0; i < MAX_BUTTONS; ++i) {
    _buttons[i].inUse = false;
    _buttons[i].state = false;
    _buttons[i].name[0] = '\0';
  }

  for (int i = 0; i < MAX_OP_PRIMARY; ++i) {
    _opPrimary[i].inUse = false;
    _opPrimary[i].cmd[0] = '\0';
    _opPrimary[i].label[0] = '\0';
  }
  for (int i = 0; i < MAX_OP_SECONDARY; ++i) {
    _opSecondary[i].inUse = false;
    _opSecondary[i].cmd[0] = '\0';
    _opSecondary[i].label[0] = '\0';
  }

  for (int i = 0; i < MAX_DEV_INPUTS; ++i) {
    _devInputs[i].inUse = false;
    _devInputs[i].name[0] = '\0';
    _devInputs[i].value = 0.0f;
  }

  _opLastCmd[0] = '\0';
}

void WebHMI::begin() {
  if (!LittleFS.begin(true)) {
    Serial.println("[WebHMI] LittleFS.begin() failed");
  }

  IPAddress ip(WEBHMI_AP_IP);
  IPAddress subnet(WEBHMI_AP_SUBNET);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, subnet);

  const bool apOk = WiFi.softAP(WEBHMI_WIFI_SSID, WEBHMI_WIFI_PASS);
  if (!apOk) {
    Serial.println("[WebHMI] WiFi.softAP failed");
  } else {
    Serial.print("[WebHMI] AP IP: ");
    Serial.println(WiFi.softAPIP());
  }

  if (_server) delete _server;
  _server = new AsyncWebServer(80);
  _setupRoutes();
  _server->begin();

  _t0 = millis();

  _cycleLastUs = micros();
  _cycleWinStartMs = millis();
  _cycleMinUs = 0;
  _cycleMaxUs = 0;
  _cycleSumUs = 0;
  _cycleCount = 0;
  _cycleMinMs = _cycleMaxMs = _cycleAvgMs = 0.0f;
}

void WebHMI::setTitle(const char* title) {
  if (title) _title = title;
}

void WebHMI::showValue(const char* name, double value) {
  if (!name || !*name) return;

  int idx = _findIndex(name);
  if (idx < 0) {
    idx = _firstFree();
    if (idx < 0) return;
    strncpy(_entries[idx].name, name, sizeof(_entries[idx].name) - 1);
    _entries[idx].name[sizeof(_entries[idx].name) - 1] = '\0';
    _entries[idx].inUse = true;
  }

  _entries[idx].isString = false;
  _entries[idx].value = value;
  _entries[idx].text[0] = '\0';
}

void WebHMI::showValue(const char* name, const char* value) {
  if (!name || !*name) return;
  if (!value) value = "";

  int idx = _findIndex(name);
  if (idx < 0) {
    idx = _firstFree();
    if (idx < 0) return;
    strncpy(_entries[idx].name, name, sizeof(_entries[idx].name) - 1);
    _entries[idx].name[sizeof(_entries[idx].name) - 1] = '\0';
    _entries[idx].inUse = true;
  }

  _entries[idx].isString = true;
  _entries[idx].value = 0.0;
  strncpy(_entries[idx].text, value, sizeof(_entries[idx].text) - 1);
  _entries[idx].text[sizeof(_entries[idx].text) - 1] = '\0';
}

void WebHMI::showValue(const char* name, const String& value) {
  showValue(name, value.c_str());
}

void WebHMI::showButton(const char* name, bool initialState) {
  if (!name || !*name) return;

  int idx = _findButtonIndex(name);
  if (idx < 0) {
    idx = _firstFreeButton();
    if (idx < 0) return;

    strncpy(_buttons[idx].name, name, sizeof(_buttons[idx].name) - 1);
    _buttons[idx].name[sizeof(_buttons[idx].name) - 1] = '\0';
    _buttons[idx].inUse = true;
    _buttons[idx].state = initialState;
  }
}

bool WebHMI::getButtonState(const char* name) const {
  if (!name || !*name) return false;
  int idx = _findButtonIndex(name);
  if (idx < 0) return false;
  return _buttons[idx].state;
}

void WebHMI::setButtonState(const char* name, bool state) {
  if (!name || !*name) return;
  int idx = _findButtonIndex(name);
  if (idx < 0) return;
  _buttons[idx].state = state;
}

// ----------------------- Operator UI: Public API ----------------------------

void WebHMI::enableOperatorUI(bool on) {
  _operatorEnabled = on;
}

int WebHMI::_firstFreeOpPrimary() const {
  for (int i = 0; i < MAX_OP_PRIMARY; ++i) if (!_opPrimary[i].inUse) return i;
  return -1;
}
int WebHMI::_firstFreeOpSecondary() const {
  for (int i = 0; i < MAX_OP_SECONDARY; ++i) if (!_opSecondary[i].inUse) return i;
  return -1;
}

void WebHMI::opAddPrimaryButton(const char* cmdId, const char* label) {
  if (!cmdId || !*cmdId) return;
  if (!label) label = cmdId;

  const int idx = _firstFreeOpPrimary();
  if (idx < 0) return;

  _opPrimary[idx].inUse = true;
  strncpy(_opPrimary[idx].cmd, cmdId, sizeof(_opPrimary[idx].cmd) - 1);
  _opPrimary[idx].cmd[sizeof(_opPrimary[idx].cmd) - 1] = '\0';
  strncpy(_opPrimary[idx].label, label, sizeof(_opPrimary[idx].label) - 1);
  _opPrimary[idx].label[sizeof(_opPrimary[idx].label) - 1] = '\0';
}

void WebHMI::opAddSecondaryButton(const char* cmdId, const char* label) {
  if (!cmdId || !*cmdId) return;
  if (!label) label = cmdId;

  const int idx = _firstFreeOpSecondary();
  if (idx < 0) return;

  _opSecondary[idx].inUse = true;
  strncpy(_opSecondary[idx].cmd, cmdId, sizeof(_opSecondary[idx].cmd) - 1);
  _opSecondary[idx].cmd[sizeof(_opSecondary[idx].cmd) - 1] = '\0';
  strncpy(_opSecondary[idx].label, label, sizeof(_opSecondary[idx].label) - 1);
  _opSecondary[idx].label[sizeof(_opSecondary[idx].label) - 1] = '\0';
}

void WebHMI::opShowText(const char* key, const char* text) {
  showValue(key, text);
}

void WebHMI::opShowText(const char* key, const String& text) {
  showValue(key, text);
}

bool WebHMI::opCommandAvailable() const {
  return (_opCmdSeq != _opCmdConsumedSeq);
}

bool WebHMI::opPopCommand(char* outCmd, size_t outCmdLen) {
  if (!outCmd || outCmdLen == 0) return false;
  if (!opCommandAvailable()) return false;

  strncpy(outCmd, _opLastCmd, outCmdLen - 1);
  outCmd[outCmdLen - 1] = '\0';

  _opCmdConsumedSeq = _opCmdSeq;
  return true;
}

// ----------------------- Cycle Stats ----------------------------------------

void WebHMI::enableCycleStats(bool on) {
  _cycleStatsEnabled = on;

  _cycleLastUs = micros();
  _cycleWinStartMs = millis();
  _cycleMinUs = 0;
  _cycleMaxUs = 0;
  _cycleSumUs = 0;
  _cycleCount = 0;
  _cycleMinMs = _cycleMaxMs = _cycleAvgMs = 0.0f;
}

void WebHMI::_cycleStatsTick() {
  if (!_cycleStatsEnabled) return;

  const uint32_t nowUs = micros();
  const uint32_t dtUs = nowUs - _cycleLastUs;
  _cycleLastUs = nowUs;

  if (dtUs > 0 && dtUs < 2000000UL) {
    if (_cycleCount == 0) {
      _cycleMinUs = dtUs;
      _cycleMaxUs = dtUs;
    } else {
      if (dtUs < _cycleMinUs) _cycleMinUs = dtUs;
      if (dtUs > _cycleMaxUs) _cycleMaxUs = dtUs;
    }
    _cycleSumUs += dtUs;
    _cycleCount++;
  }

  const uint32_t nowMs = millis();
  if (nowMs - _cycleWinStartMs >= 1000UL) {
    if (_cycleCount > 0) {
      _cycleMinMs = (float)_cycleMinUs / 1000.0f;
      _cycleMaxMs = (float)_cycleMaxUs / 1000.0f;
      _cycleAvgMs = (float)((double)_cycleSumUs / (double)_cycleCount) / 1000.0f;
    } else {
      _cycleMinMs = _cycleMaxMs = _cycleAvgMs = 0.0f;
    }

    _cycleWinStartMs = nowMs;
    _cycleMinUs = 0;
    _cycleMaxUs = 0;
    _cycleSumUs = 0;
    _cycleCount = 0;
  }
}

// ----------------------- Dev Inputs -----------------------------------------

int WebHMI::_findDevInputIndex(const char* name) const {
  for (int i = 0; i < MAX_DEV_INPUTS; ++i) {
    if (!_devInputs[i].inUse) continue;
    if (strcmp(_devInputs[i].name, name) == 0) return i;
  }
  return -1;
}

int WebHMI::_firstFreeDevInput() const {
  for (int i = 0; i < MAX_DEV_INPUTS; ++i) {
    if (!_devInputs[i].inUse) return i;
  }
  return -1;
}

void WebHMI::showDevInputFloat(const char* name, float defaultValue) {
  if (!name || !*name) return;

  int idx = _findDevInputIndex(name);
  if (idx < 0) {
    idx = _firstFreeDevInput();
    if (idx < 0) return;

    _devInputs[idx].inUse = true;
    strncpy(_devInputs[idx].name, name, sizeof(_devInputs[idx].name) - 1);
    _devInputs[idx].name[sizeof(_devInputs[idx].name) - 1] = '\0';
    _devInputs[idx].value = defaultValue;
  }
  // If exists: do not overwrite value.
}

float WebHMI::getValue(const char* name, float fallback) const {
  if (!name || !*name) return fallback;
  const int idx = _findDevInputIndex(name);
  if (idx < 0) return fallback;
  return _devInputs[idx].value;
}

// ----------------------------------------------------------------------------

void WebHMI::enableDemo(bool on) {
  _demoOn = on;
  _demoMs = millis();
}

void WebHMI::tick() {
  _cycleStatsTick();

  if (_demoOn) _demoStep();

  if (_isLogging) {
    const unsigned long now = millis();
    if ((now - _lastLogMs) >= kLogIntervalMs) {
      _lastLogMs = now;
      _writeRow();
      _flushIfOpen();
    }
  }
}

int WebHMI::_findIndex(const char* name) {
  for (int i = 0; i < MAX_ENTRIES; ++i) {
    if (!_entries[i].inUse) continue;
    if (strcmp(_entries[i].name, name) == 0) return i;
  }
  return -1;
}

int WebHMI::_firstFree() {
  for (int i = 0; i < MAX_ENTRIES; ++i) {
    if (!_entries[i].inUse) return i;
  }
  return -1;
}

int WebHMI::_findButtonIndex(const char* name) const {
  for (int i = 0; i < MAX_BUTTONS; ++i) {
    if (!_buttons[i].inUse) continue;
    if (strcmp(_buttons[i].name, name) == 0) return i;
  }
  return -1;
}

int WebHMI::_firstFreeButton() const {
  for (int i = 0; i < MAX_BUTTONS; ++i) {
    if (!_buttons[i].inUse) return i;
  }
  return -1;
}

// ----------------------------- Logging --------------------------------------

void WebHMI::_startLogging() {
  if (_isLogging) return;

  _logFile = LittleFS.open(WEBHMI_LOG_PATH, "w");
  if (!_logFile) {
    Serial.println("[WebHMI] Logging: open failed");
    _isLogging = false;
    return;
  }

  _isLogging = true;
  _headerWritten = false;
  _logStartMs = millis();
  _lastLogMs = 0;

  _writeHeader();
}

void WebHMI::_stopLogging(bool fromAuto) {
  (void)fromAuto;
  if (!_isLogging) return;

  _flushIfOpen();
  if (_logFile) _logFile.close();

  _isLogging = false;
}

void WebHMI::_writeHeader() {
  if (!_logFile || _headerWritten) return;

  _logFile.print("ms");
  for (int i = 0; i < MAX_ENTRIES; ++i) {
    if (!_entries[i].inUse) continue;
    _logFile.print(';');
    printCsvEscaped(_logFile, _entries[i].name);
  }
  for (int i = 0; i < MAX_BUTTONS; ++i) {
    if (!_buttons[i].inUse) continue;
    _logFile.print(';');
    printCsvEscaped(_logFile, _buttons[i].name);
  }
  _logFile.print("\r\n");
  _headerWritten = true;
}

void WebHMI::_writeRow() {
  if (!_logFile || !_isLogging) return;

  const unsigned long ms = millis() - _logStartMs;
  _logFile.print(ms);

  for (int i = 0; i < MAX_ENTRIES; ++i) {
    if (!_entries[i].inUse) continue;
    _logFile.print(';');

    if (_entries[i].isString) {
      printCsvEscaped(_logFile, _entries[i].text);
    } else {
      printWithComma(_logFile, _entries[i].value, 3);
    }
  }

  for (int i = 0; i < MAX_BUTTONS; ++i) {
    if (!_buttons[i].inUse) continue;
    _logFile.print(';');
    _logFile.print(_buttons[i].state ? "1" : "0");
  }

  _logFile.print("\r\n");
}

void WebHMI::_flushIfOpen() {
  if (_logFile) _logFile.flush();
}

// ----------------------------- Web Routes -----------------------------------

void WebHMI::_setupRoutes() {
  _server->on("/", HTTP_GET, [this](AsyncWebServerRequest* req){
    req->send(200, "text/html; charset=utf-8", _buildHtml());
  });

  _server->on("/values", HTTP_GET, [this](AsyncWebServerRequest* req){
    req->send(200, "application/json", _buildJson());
  });

  _server->on("/start", HTTP_GET, [this](AsyncWebServerRequest* req){
    _startLogging();
    req->send(200, "application/json", _isLogging ? "{\"ok\":true}" : "{\"ok\":false}");
  });

  _server->on("/stop", HTTP_GET, [this](AsyncWebServerRequest* req){
    _stopLogging();
    req->send(200, "application/json", "{\"ok\":true}");
  });

  _server->on("/download", HTTP_GET, [this](AsyncWebServerRequest* req){
    if (!LittleFS.exists(WEBHMI_LOG_PATH)) {
      req->send(404, "text/plain", "log not found");
      return;
    }
    AsyncWebServerResponse* r = req->beginResponse(LittleFS, WEBHMI_LOG_PATH, "text/csv");
    if (!r) {
      String msg = "RESP_NULL heap=" + String((unsigned)ESP.getFreeHeap());
      req->send(500, "text/plain", msg);
      return;
    }
    r->addHeader("Content-Disposition", "attachment; filename=log.csv");
    r->addHeader("Cache-Control", "no-store");
    req->send(r);
  });

  _server->on("/toggle", HTTP_GET, [this](AsyncWebServerRequest* req){
    if (!req->hasParam("name")) {
      req->send(400, "application/json", "{\"ok\":false,\"err\":\"missing name\"}");
      return;
    }
    const String n = req->getParam("name")->value();
    const int idx = _findButtonIndex(n.c_str());
    if (idx < 0) {
      req->send(404, "application/json", "{\"ok\":false,\"err\":\"unknown button\"}");
      return;
    }
    _buttons[idx].state = !_buttons[idx].state;
    req->send(200, "application/json", "{\"ok\":true}");
  });

  _server->on("/op", HTTP_GET, [this](AsyncWebServerRequest* req){
    if (!req->hasParam("cmd")) {
      req->send(400, "application/json", "{\"ok\":false,\"err\":\"missing cmd\"}");
      return;
    }
    const String cmd = req->getParam("cmd")->value();
    strncpy(_opLastCmd, cmd.c_str(), sizeof(_opLastCmd) - 1);
    _opLastCmd[sizeof(_opLastCmd) - 1] = '\0';
    _opCmdSeq++;
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // Dev input setter
  _server->on("/setInput", HTTP_GET, [this](AsyncWebServerRequest* req){
    if (!req->hasParam("name") || !req->hasParam("value")) {
      req->send(400, "application/json", "{\"ok\":false,\"err\":\"missing name/value\"}");
      return;
    }

    const String name = req->getParam("name")->value();
    const String valS = req->getParam("value")->value();

    const int idx = _findDevInputIndex(name.c_str());
    if (idx < 0) {
      req->send(404, "application/json", "{\"ok\":false,\"err\":\"unknown input\"}");
      return;
    }

    if (!isValidFloatString(valS)) {
      req->send(400, "application/json", "{\"ok\":false,\"err\":\"invalid\"}");
      return;
    }

    const float v = parseFloatCommaDot(valS);
    _devInputs[idx].value = v;
    req->send(200, "application/json", "{\"ok\":true}");
  });
}

// ----------------------------- JSON Builder ---------------------------------

String WebHMI::_buildJson() const {
  String out = F("{\"values\":[");
  bool first = true;

  for (int i = 0; i < MAX_ENTRIES; ++i) {
    if (!_entries[i].inUse) continue;
    if (!first) out += ',';
    first = false;

    out += F("{\"name\":\"");
    jsonAppendEscaped(out, _entries[i].name);
    out += F("\",\"value\":");

    if (_entries[i].isString) {
      out += '\"';
      jsonAppendEscaped(out, _entries[i].text);
      out += '\"';
    } else {
      out += String(_entries[i].value, 3);
    }

    out += '}';
  }

  out += F("],\"buttons\":[");
  first = true;

  for (int i = 0; i < MAX_BUTTONS; ++i) {
    if (!_buttons[i].inUse) continue;
    if (!first) out += ',';
    first = false;

    out += F("{\"name\":\"");
    jsonAppendEscaped(out, _buttons[i].name);
    out += F("\",\"state\":");
    out += (_buttons[i].state ? F("true") : F("false"));
    out += '}';
  }

  // Dev inputs
  out += F("],\"inputs\":[");
  first = true;
  for (int i = 0; i < MAX_DEV_INPUTS; ++i) {
    if (!_devInputs[i].inUse) continue;
    if (!first) out += ',';
    first = false;

    out += F("{\"name\":\"");
    jsonAppendEscaped(out, _devInputs[i].name);
    out += F("\",\"value\":");
    out += String(_devInputs[i].value, 6);
    out += '}';
  }
  out += F("]");

  out += F(",\"logging\":");
  out += (_isLogging ? F("true") : F("false"));
  out += F(",\"elapsed_ms\":");
  out += (_isLogging ? String(millis() - _logStartMs) : String(0));

  out += F(",\"operator\":");
  out += (_operatorEnabled ? F("true") : F("false"));

  out += F(",\"cycle\":{");
  out += F("\"enabled\":"); out += (_cycleStatsEnabled ? F("true") : F("false"));
  out += F(",\"min_ms\":"); out += String(_cycleMinMs, 3);
  out += F(",\"max_ms\":"); out += String(_cycleMaxMs, 3);
  out += F(",\"avg_ms\":"); out += String(_cycleAvgMs, 3);
  out += F("}");

  out += '}';

  return out;
}

// ----------------------------- HTML Builder ---------------------------------

String WebHMI::_buildHtml() const {
  // Operator config JSON
  String opCfg = F("{\"enabled\":");
  opCfg += (_operatorEnabled ? F("true") : F("false"));

  opCfg += F(",\"primary\":[");
  bool first = true;
  for (int i = 0; i < MAX_OP_PRIMARY; ++i) {
    if (!_opPrimary[i].inUse) continue;
    if (!first) opCfg += ',';
    first = false;
    opCfg += F("{\"cmd\":\""); jsonAppendEscaped(opCfg, _opPrimary[i].cmd);
    opCfg += F("\",\"label\":\""); jsonAppendEscaped(opCfg, _opPrimary[i].label);
    opCfg += F("\"}");
  }

  opCfg += F("],\"secondary\":[");
  first = true;
  for (int i = 0; i < MAX_OP_SECONDARY; ++i) {
    if (!_opSecondary[i].inUse) continue;
    if (!first) opCfg += ',';
    first = false;
    opCfg += F("{\"cmd\":\""); jsonAppendEscaped(opCfg, _opSecondary[i].cmd);
    opCfg += F("\",\"label\":\""); jsonAppendEscaped(opCfg, _opSecondary[i].label);
    opCfg += F("\"}");
  }

  opCfg += F("]}");

  // Dev inputs config JSON
  String inCfg = F("{\"inputs\":[");
  first = true;
  for (int i = 0; i < MAX_DEV_INPUTS; ++i) {
    if (!_devInputs[i].inUse) continue;
    if (!first) inCfg += ',';
    first = false;
    inCfg += F("{\"name\":\""); jsonAppendEscaped(inCfg, _devInputs[i].name);
    inCfg += F("\"}");
  }
  inCfg += F("]}");

  String html;
  html.reserve(12000);

  html += F("<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>"
            "<title>");
  jsonAppendEscaped(html, _title.c_str());
  html += F("</title>");

  html += F("<style>"
            ":root{--bg:#0b0f17;--card:#121a27;--muted:#9aa4b2;--text:#e7edf5;--acc:#4ea1ff;--warn:#ffb020;--err:#ff5a5f;--ok:#35d07f;}"
            "html,body{margin:0;padding:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Cantarell,Noto Sans,sans-serif;}"
            ".wrap{max-width:980px;margin:0 auto;padding:16px;}"
            ".topbar{display:flex;align-items:center;justify-content:space-between;gap:12px;}"
            ".leftTop{display:flex;align-items:center;gap:10px;min-width:0;}"
            ".title{font-size:18px;font-weight:700;letter-spacing:.2px;user-select:none;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}"
            ".rightTop{display:flex;align-items:center;gap:10px;}"
            ".pill{font-size:12px;color:var(--muted);padding:6px 10px;border:1px solid rgba(255,255,255,.08);border-radius:999px;}"
            ".card{background:var(--card);border:1px solid rgba(255,255,255,.06);border-radius:16px;padding:14px;box-shadow:0 6px 20px rgba(0,0,0,.25);}"
            ".grid{display:grid;gap:12px;}"
            ".grid2{grid-template-columns:repeat(2,minmax(0,1fr));}"
            "@media(max-width:720px){.grid2{grid-template-columns:1fr;}}"
            ".status{display:flex;flex-direction:column;gap:6px;}"
            ".label{font-size:12px;color:var(--muted);}"
            ".value{font-size:16px;font-weight:650;line-height:1.25;}"
            ".error{border-color:rgba(255,90,95,.35);}"
            ".hint{border-color:rgba(255,176,32,.25);}"
            ".btnRow{display:flex;flex-wrap:wrap;gap:10px;}"
            "button{appearance:none;border:0;border-radius:14px;padding:14px 16px;font-size:15px;font-weight:650;cursor:pointer;}"
            ".btnPrimary{background:linear-gradient(180deg,rgba(78,161,255,.95),rgba(78,161,255,.75));color:#07111f;}"
            ".btnSecondary{background:rgba(255,255,255,.06);color:var(--text);border:1px solid rgba(255,255,255,.09);}"
            ".btnDanger{background:linear-gradient(180deg,rgba(255,90,95,.95),rgba(255,90,95,.75));color:#190406;}"
            ".btnTiny{padding:10px 12px;font-size:13px;border-radius:12px;}"
            ".table{width:100%;border-collapse:collapse;font-size:13px;}"
            ".table td{padding:8px 10px;border-bottom:1px solid rgba(255,255,255,.06);vertical-align:top;}"
            ".k{color:var(--muted);}"
            ".sectionTitle{font-size:13px;color:var(--muted);margin:14px 0 8px;}"
            ".hidden{display:none;}"
            "input[type='text']{width:100%;box-sizing:border-box;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.10);color:var(--text);border-radius:12px;padding:10px 12px;font-size:13px;outline:none;}"
            ".toast{position:fixed;left:50%;bottom:14px;transform:translateX(-50%);background:rgba(0,0,0,.65);border:1px solid rgba(255,255,255,.12);padding:10px 12px;border-radius:12px;font-size:13px;color:var(--text);opacity:0;transition:opacity .2s;}"
            ".toast.show{opacity:1;}"
            "</style></head><body>");

  html += F("<div class='wrap'>"
            "<div class='topbar'>"
              "<div class='leftTop'>"
                "<div class='title' id='title'>");
  html += _title;
  html += F("</div>"
                "<div class='pill' id='conn'>AP</div>"
              "</div>"
              "<div class='rightTop'>"
                "<button id='devSwitch' class='btnSecondary btnTiny'>Dev</button>"
              "</div>"
            "</div>");

  html += F("<div id='operatorRoot' class='grid' style='margin-top:12px;'>"
              "<div class='card status' id='stateCard'>"
                "<div class='label'>State</div>"
                "<div class='value' id='stateText'>-</div>"
                "<div class='label hidden' id='errorLabel'>Error</div>"
                "<div class='value hidden' id='errorText'>-</div>"
              "</div>"
              "<div class='card'>"
                "<div class='sectionTitle'>Actions</div>"
                "<div class='btnRow' id='primaryBtns'></div>"
              "</div>"
            "</div>");

  html += F("<div id='devRoot' class='hidden' style='margin-top:12px;'>"
              "<div class='card'>"
                "<div class='sectionTitle'>Developer</div>"
                "<div class='btnRow'>"
                  "<button class='btnSecondary btnTiny' onclick='logStart()'>Log Start</button>"
                  "<button class='btnSecondary btnTiny' onclick='logStop()'>Log Stop</button>"
                  "<button class='btnSecondary btnTiny' onclick='logDownload()'>Download</button>"
                "</div>"
              "</div>"

              "<div class='card hidden' id='inputCard' style='margin-top:12px;'>"
                "<div class='sectionTitle'>Inputs</div>"
                "<table class='table' id='inputTable'></table>"
              "</div>"

              "<div class='card' style='margin-top:12px;'>"
                "<div class='sectionTitle'>Values</div>"
                "<table class='table' id='valTable'></table>"
              "</div>"
              "<div class='card' style='margin-top:12px;'>"
                "<div class='sectionTitle'>Buttons</div>"
                "<div class='btnRow' id='devBtns'></div>"
              "</div>"
              "<div class='card hidden' id='cycleCard' style='margin-top:12px;'>"
                "<div class='sectionTitle'>Cycle Time</div>"
                "<table class='table'>"
                  "<tr><td class='k'>min</td><td id='cyMin'>-</td></tr>"
                  "<tr><td class='k'>max</td><td id='cyMax'>-</td></tr>"
                  "<tr><td class='k'>avg</td><td id='cyAvg'>-</td></tr>"
                "</table>"
              "</div>"
            "</div>");

  html += F("</div><div class='toast' id='toast'></div>");

  html += F("<script>"
            "const OP_CFG = ");
  html += opCfg;
  html += F("; const IN_CFG = ");
  html += inCfg;
  html += F(";"

            "let devShown=false;"
            "const devBtn=document.getElementById('devSwitch');"
            "devBtn.addEventListener('click',()=>{toggleDev();});"

            "function toggleDev(){"
              "devShown=!devShown;"
              "document.getElementById('devRoot').classList.toggle('hidden',!devShown);"
              "devBtn.textContent = devShown ? 'Operator' : 'Dev';"
              "toast(devShown?'Developer panel shown':'Developer panel hidden');"
            "}"

            "function toast(msg){"
              "const t=document.getElementById('toast');"
              "t.textContent=msg;"
              "t.classList.add('show');"
              "setTimeout(()=>t.classList.remove('show'),900);"
            "}"

            "function esc(s){return (''+s).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;');}"

            "function buildButtons(){"
              "const p=document.getElementById('primaryBtns');"
              "p.innerHTML='';"
              "if(!OP_CFG.enabled){document.getElementById('operatorRoot').classList.add('hidden');return;}"
              "OP_CFG.primary.forEach((b)=>{"
                "const btn=document.createElement('button');"
                "btn.className='btnPrimary';"
                "btn.textContent=b.label;"
                "btn.onclick=()=>opSend(b.cmd);"
                "p.appendChild(btn);"
              "});"
              "Array.from(p.children).forEach(btn=>{"
                "const t=btn.textContent.toLowerCase();"
                "if(t.includes('e-stop')||t.includes('estop'))btn.className='btnDanger';"
              "});"
            "}"

            "async function opSend(cmd){"
              "try{await fetch('/op?cmd='+encodeURIComponent(cmd));toast('Sent: '+cmd);}catch(e){toast('Send failed');}"
            "}"

            "async function logStart(){await fetch('/start');toast('Log start');}"
            "async function logStop(){await fetch('/stop');toast('Log stop');}"
            "function logDownload(){window.location='/download';}"
            "async function toggleDevBtn(name){await fetch('/toggle?name='+encodeURIComponent(name));}"

            "function pickValue(values,key){for(const v of values){if(v.name===key)return v.value;}return '';}"
            "function pickInput(inputs,key){for(const v of inputs){if(v.name===key)return v.value;}return null;}"
            "function isOpCustomKey(k){return k.startsWith('OPTXT.') || k.startsWith('UI.Custom.');}"

            "function renderOperator(values){"
              "const st=pickValue(values,'UI.StateText');"
              "const et=pickValue(values,'UI.ErrorText');"
              "document.getElementById('stateText').textContent=st||'-';"
              "const hasErr=(et && (''+et).trim().length>0);"
              "document.getElementById('errorLabel').classList.toggle('hidden',!hasErr);"
              "document.getElementById('errorText').classList.toggle('hidden',!hasErr);"
              "document.getElementById('errorText').textContent=hasErr?et:'-';"
            "}"

            "let inputDebounceTimers={};"
            "function buildInputs(){"
              "const card=document.getElementById('inputCard');"
              "const tbl=document.getElementById('inputTable');"
              "tbl.innerHTML='';"
              "if(!IN_CFG.inputs || IN_CFG.inputs.length===0){card.classList.add('hidden');return;}"
              "card.classList.remove('hidden');"
              "for(const it of IN_CFG.inputs){"
                "const tr=document.createElement('tr');"
                "const tdK=document.createElement('td');"
                "tdK.className='k';"
                "tdK.textContent=it.name;"
                "const tdV=document.createElement('td');"
                "const inp=document.createElement('input');"
                "inp.type='text';"
                "inp.inputMode='decimal';"
                "inp.autocomplete='off';"
                "inp.spellcheck=false;"
                "inp.id='in_'+it.name;"
                "inp.placeholder='0,0';"
                "inp.addEventListener('input',()=>{"
                  "const s=inp.value;"
                  "const ok=/^[+-]?[0-9]*[\\.,]?[0-9]*$/.test(s) && /[0-9]/.test(s);"
                  "if(!ok){"
                    "inp.value = inp.value.replace(/[^0-9\\.,\\-\\+]/g,'');"
                    "return;"
                  "}"
                  "clearTimeout(inputDebounceTimers[inp.id]);"
                  "inputDebounceTimers[inp.id]=setTimeout(()=>sendInput(it.name, inp.value), 450);"
                "});"
                "tdV.appendChild(inp);"
                "tr.appendChild(tdK);"
                "tr.appendChild(tdV);"
                "tbl.appendChild(tr);"
              "}"
            "}"

            "async function sendInput(name,val){"
              "try{"
                "const r=await fetch('/setInput?name='+encodeURIComponent(name)+'&value='+encodeURIComponent(val));"
                "const j=await r.json();"
                "if(!j.ok){toast('Input rejected');}"
              "}catch(e){toast('Input send failed');}"
            "}"

            "function renderDev(values,buttons,cycle,inputs){"
              "const vt=document.getElementById('valTable');"
              "let rows='';"
              "for(const v of values){rows += `<tr><td class='k'>${esc(v.name)}</td><td>${esc(v.value)}</td></tr>`;}"
              "vt.innerHTML=rows;"
              "const db=document.getElementById('devBtns');"
              "db.innerHTML='';"
              "for(const b of buttons){"
                "const btn=document.createElement('button');"
                "btn.className='btnSecondary btnTiny';"
                "btn.textContent=b.name+': '+(b.state?'ON':'OFF');"
                "btn.onclick=()=>toggleDevBtn(b.name);"
                "db.appendChild(btn);"
              "}"
              "const c=document.getElementById('cycleCard');"
              "const showCycle = !!(cycle && cycle.enabled);"
              "c.classList.toggle('hidden', !showCycle);"
              "if(showCycle){"
                "document.getElementById('cyMin').textContent = (cycle.min_ms ?? '-') + ' ms';"
                "document.getElementById('cyMax').textContent = (cycle.max_ms ?? '-') + ' ms';"
                "document.getElementById('cyAvg').textContent = (cycle.avg_ms ?? '-') + ' ms';"
              "}"
              "if(inputs && IN_CFG.inputs){"
                "for(const it of IN_CFG.inputs){"
                  "const el=document.getElementById('in_'+it.name);"
                  "if(!el) continue;"
                  "if(document.activeElement===el) continue;"
                  "const v=pickInput(inputs,it.name);"
                  "if(v===null) continue;"
                  "const s=(''+v);"
                  "if(el.value!==s) el.value=s;"
                "}"
              "}"
            "}"

            "async function poll(){"
              "try{"
                "const r=await fetch('/values',{cache:'no-store'});"
                "const j=await r.json();"
                "document.getElementById('conn').textContent='Online';"
                "if(OP_CFG.enabled){renderOperator(j.values||[]);} "
                "if(devShown){renderDev(j.values||[], j.buttons||[], j.cycle||null, j.inputs||[]);} "
              "}catch(e){document.getElementById('conn').textContent='Offline';}"
            "}"

            "buildButtons();"
            "buildInputs();"
            "if(!OP_CFG.enabled){document.getElementById('operatorRoot').classList.add('hidden');}"
            "setInterval(poll,250);poll();"
            "</script></body></html>");

  return html;
}

// ----------------------------- Demo -----------------------------------------

void WebHMI::_demoStep() {
  const uint32_t now = millis();
  if (now - _demoMs < 250) return;
  _demoMs = now;

  _cntA++;
  _cntB += 0.1f;

  showValue("demo.counterA", (double)_cntA);
  showValue("demo.counterB", (double)_cntB);

  showButton("demo.toggle", false);
  showValue("UI.StateText", "Demo running");
  showValue("UI.ErrorText", "");
}
