#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include "ArduinoJson.h"

// ===== WiFi credentials for AP mode =====
const char* ssid = "SWAS2026";
const char* password = "12345678";

// ===== MAC address of ESP32-Dev Module ===== 
// MAKE SURE THIS MATCHES YOUR ROBOT'S MAC ADDRESS EXACTLY
uint8_t devModuleMac[] = {0x5C, 0x01, 0x3B, 0x9C, 0x31, 0xC4}; // {0x5C, 0x01, 0x3B, 0x9C, 0x31, 0xC4}  {0xF4, 0x65, 0x0B, 0x42, 0x06, 0xF4}

// ===== Structures =====
typedef struct {
  uint16_t us_front, us_back, us_left, us_right, us_incline;
  float imu_roll, imu_pitch, imu_yaw;
  double gps_lat, gps_lon;
  bool gps_valid;
  uint32_t timestamp;
  float local_N_to_monument;
  float local_E_to_monument;
  float gps_distance_to_monument;
  int8_t nav_command; 
} struct_message;

typedef struct {
  char command[16];
} CommandMessage;

#define LED_PIN 4

// ===== Globals =====
struct_message sensorData;
CommandMessage commandMsg;
WebServer server(80);
unsigned long lastDataReceived = 0;

// ===== ESP-NOW Receive Callback =====
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(struct_message)) {
    memcpy(&sensorData, incomingData, sizeof(sensorData));
    lastDataReceived = millis();
  }
}

// ===== Web Handlers =====
void handleRoot();
void handleData();
void handleCommand();

// --- HTML / JS DASHBOARD ---
void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html lang="en">
  <head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Robot Dog Control</title>
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600;800&family=JetBrains+Mono:wght@400;600&family=Orbitron:wght@600;800&display=swap" rel="stylesheet">
  <style>
    :root{
      --bg:#0b0f14; --bg2:#0f141c; --card:#101824; --border:#1b2635;
      --muted:#9aa4b2; --text:#e7edf4; --neon:#ff3636;
      --neon-glow:0 0 12px rgba(255,54,54,.55), 0 0 28px rgba(255,54,54,.35);
      --radius:14px; --shadow:0 10px 30px rgba(0,0,0,.35);
    }
    *{ box-sizing:border-box }
    body{ margin:0; background: var(--bg); color:var(--text); font-family: 'Inter', sans-serif; }
    .header{ position:sticky; top:0; z-index:5; background:rgba(11,15,20,.9); border-bottom:1px solid var(--border); backdrop-filter: blur(6px); }
    .header-inner{ max-width:1220px; margin:auto; padding:15px; display:flex; align-items:center; justify-content:center; }
    .title{ font-family:'Orbitron',sans-serif; font-weight:800; font-size:22px; color:var(--neon); text-shadow:var(--neon-glow); }
    .wrap{ max-width:1220px; margin:20px auto; padding:0 16px }
    .grid{ display:grid; gap:12px; grid-template-columns: repeat(2, 1fr); }
    .card{background:linear-gradient(180deg,var(--card),var(--bg2));border:1px solid var(--border);border-radius:var(--radius);box-shadow:var(--shadow);display:flex;flex-direction:column;}
    .card .head{padding:10px 12px;border-bottom:1px solid var(--border);}
    .card h3{margin:0;font-size:13px;letter-spacing:.6px;text-transform:uppercase;font-family:'Orbitron',sans-serif;color:var(--neon);}
    .card .body{padding:12px;}
    .kv{display:grid;grid-template-columns:repeat(2, 1fr);gap:8px 10px;font-family:'JetBrains Mono',monospace;font-size:13px}
    .kv label{color:var(--muted)} .kv .val{font-weight:800;color:#ffd3d3;}
    .nav-header { grid-column: 1 / span 2; font-weight: 600; color: var(--text); margin-top: 6px; border-bottom: 1px dashed var(--border); padding-bottom: 2px; }
    .control-grid{display:grid;grid-template-columns:repeat(auto-fit, minmax(180px, 1fr));gap:10px;}
    .ctrl-btn{height:48px;border-radius:12px;border:1px solid var(--border);background:#1a2533;color:var(--text);font-size:15px;font-weight:800;cursor:pointer;transition:all 0.2s ease;font-family:'Orbitron',sans-serif;}
    .ctrl-btn:hover{border-color:#ff3636;}
    .ctrl-btn.active{background:var(--neon);color:#000;}
    @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
  </style>
  </head>
  <body>
    <div class="header"><div class="header-inner"><div class="title">Robot Dog Control</div></div></div>
    <div class="wrap">
      <div class="grid">
        <section class="card">
          <div class="head"><h3>Ultrasonic Sensors (CM)</h3></div>
          <div class="body"><div class="kv">
            <label>Front</label><div id="usFront" class="val">0</div>
            <label>Left</label><div id="usLeft" class="val">0</div>
            <label>Right</label><div id="usRight" class="val">0</div>
            <label>Back</label><div id="usBack" class="val">0</div>
            <label>Incline</label><div id="usIncline" class="val">0</div>
          </div></div>
        </section>

        <section class="card">
          <div class="head"><h3>Navigation</h3></div>
          <div class="body"><div class="kv">
            <div class="nav-header">STATUS</div>
            <label>Distance</label><div id="distToMon" class="val">--</div>
            <div class="nav-header">RAW GPS</div>
            <label>Latitude</label><div id="gpsLat" class="val">--</div>
            <label>Longitude</label><div id="gpsLon" class="val">--</div>
            <div class="nav-header">DIRECTION</div>
            <label>CMD</label><div id="navStatus" class="val" style="color:#00FF00;">--</div>
          </div></div>
        </section>

        <section class="card">
          <div class="head"><h3>IMU Orientation (°)</h3></div>
          <div class="body"><div class="kv">
            <label>Roll</label><div id="imuRoll" class="val">0.0</div>
            <label>Pitch</label><div id="imuPitch" class="val">0.0</div>
            <label>Yaw</label><div id="imuYaw" class="val">0.0</div>
          </div></div>
        </section>

        <section class="card">
          <div class="head"><h3>Controls</h3></div>
          <div class="body"><div class="control-grid">
            <button class="ctrl-btn" style="border-color:#34b7ff; color:#34b7ff;" onclick="sendCommand('INIT')">INITIALIZE</button>
            <button class="ctrl-btn" onclick="sendCommand('POST')">POST</button>
            <button class="ctrl-btn" onclick="sendCommand('H_BUILDING')">H_BUILDING</button>
            <button id="btnStop" class="ctrl-btn" onclick="toggleStop()">STOP</button>
          </div></div>
        </section>
      </div>
    </div>
  <script>
  let isPaused = false;
  const us = ['usFront','usLeft','usRight','usBack','usIncline'].map(id => document.getElementById(id));
  const imuRoll = document.getElementById('imuRoll'), imuPitch = document.getElementById('imuPitch'), imuYaw = document.getElementById('imuYaw');
  const gpsLat  = document.getElementById('gpsLat'),  gpsLon  = document.getElementById('gpsLon');
  const distToMon = document.getElementById('distToMon'), navStatus = document.getElementById('navStatus');

  function toggleStop() {
    const btn = document.getElementById('btnStop');
    sendCommand('STOP'); 
    isPaused = !isPaused;
    if (isPaused) {
      btn.innerText = "RESUME"; btn.style.background = "#27ae60"; btn.style.color = "#fff";
    } else {
      btn.innerText = "STOP"; btn.style.background = ""; btn.style.color = "";
    }
  }
  function sendCommand(cmd){
    fetch('/command?cmd=' + cmd).catch(()=>{});
  }
  function updateData(){
    fetch('/data').then(r=>r.json()).then(d=>{
      us[0].textContent = d.us_front; us[1].textContent = d.us_left;
      us[2].textContent = d.us_right; us[3].textContent = d.us_back;
      us[4].textContent = d.us_incline;
      imuRoll.textContent = d.imu_roll.toFixed(1); imuPitch.textContent = d.imu_pitch.toFixed(1); imuYaw.textContent = d.imu_yaw.toFixed(1);
      distToMon.textContent = (d.gps_dist !== undefined && d.gps_dist >= 0) ? d.gps_dist.toFixed(1) : '--';
      if (d.gps_valid){ gpsLat.textContent = d.gps_lat; gpsLon.textContent = d.gps_lon; } 
      else { gpsLat.textContent = '--'; gpsLon.textContent = '--'; }
      const cmdMap = ["WAIT", "R_LEFT", "R_RIGHT", "FRWD", "FAR", "W_ORD", "S_LEFT", "S_RIGHT"];
      navStatus.textContent = (d.nav_cmd !== undefined && d.nav_cmd >= 0) ? cmdMap[d.nav_cmd] : "--";
    }).catch(()=>{});
  }
  window.onload = () => { setInterval(updateData, 500); };
  </script>
  </body></html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleData() {
  JsonDocument doc;
  doc["us_front"] = sensorData.us_front; doc["us_back"] = sensorData.us_back;
  doc["us_left"] = sensorData.us_left; doc["us_right"] = sensorData.us_right;
  doc["us_incline"] = sensorData.us_incline;
  doc["imu_roll"] = sensorData.imu_roll; doc["imu_pitch"] = sensorData.imu_pitch; doc["imu_yaw"] = sensorData.imu_yaw;
  doc["gps_lat"] = serialized(String(sensorData.gps_lat, 6)); doc["gps_lon"] = serialized(String(sensorData.gps_lon, 6));
  doc["gps_valid"] = sensorData.gps_valid; doc["gps_dist"] = sensorData.gps_distance_to_monument;
  doc["nav_cmd"] = sensorData.nav_command; 
  String output; serializeJson(doc, output); server.send(200, "application/json", output);
}

void handleCommand() {
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd");
    
    memset(&commandMsg, 0, sizeof(commandMsg));
    cmd.toCharArray(commandMsg.command, sizeof(commandMsg));
    
    esp_err_t result = esp_now_send(devModuleMac, (uint8_t *)&commandMsg, sizeof(commandMsg));
    
    if (result == ESP_OK) {
        Serial.println(">>> Command Sent Successfully: " + cmd);
    } else {
        Serial.print(">>> Error Sending Command: " + cmd);
        Serial.print(" | Error Code: ");
        Serial.println(result);
    }

    server.send(200, "text/plain", "OK");
  }
}

void setup() {
  Serial.begin(115200);
  
  // Set WiFi to AP+STA
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, devModuleMac, 6);
  peerInfo.channel = 0; 
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer added successfully");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/command", handleCommand);
  server.begin();
  
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}