#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include "esp_camera.h"

// ==== Wi-Fi AP credentials ====
const char* ssid = "SWAS2026";
const char* password = "12345678";

// ==== Custom IP Configuration ====
IPAddress local_IP(192, 168, 5, 1);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);

// ==== Receiver MAC (ESP32 DEV MODULE) ====
uint8_t receiverMac[] = {0x5C, 0x01, 0x3B, 0x9C, 0x79, 0xF0}; 

WebServer server(80);

typedef struct { char cmd[20]; } espMessage_t;
espMessage_t message;

// ==== Camera pin mapping (AI Thinker) ====
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ==== Capture image ====
void handleCapture() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// ==== Professional Web Page ====
void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html lang="en">
  <head>
    <meta charset='UTF-8'>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Team-5: MMD-1 Case Study</title>
    <style>
      :root {
        --primary: #003366;    /* Professional Academic Blue */
        --accent: #ff9900;     /* Highlight Color */
        --bg: #f4f6f9;         /* Light Grey Background */
        --card: #ffffff;
        --text-main: #333;
        --text-muted: #666;
      }
      body {
        font-family: 'Segoe UI', Helvetica, Arial, sans-serif;
        background: var(--bg);
        color: var(--text-main);
        margin: 0;
        padding: 20px;
        display: flex;
        justify-content: center;
        min-height: 100vh;
      }
      .container {
        background: var(--card);
        width: 100%;
        max-width: 700px;
        border-radius: 12px;
        box-shadow: 0 10px 30px rgba(0,0,0,0.08);
        overflow: hidden;
        text-align: center;
        border-top: 6px solid var(--primary);
      }
      
      /* --- INFO SECTION (Top) --- */
      .header-section {
        padding: 30px 25px 20px;
        border-bottom: 1px solid #eee;
      }
      h1 {
        color: var(--primary);
        margin: 0;
        font-size: 2.2rem;
        letter-spacing: -0.5px;
      }
      h2 {
        color: var(--text-muted);
        font-size: 1.1rem;
        font-weight: 500;
        text-transform: uppercase;
        margin: 5px 0 15px;
        letter-spacing: 1px;
      }
      .course-info {
        font-weight: 600;
        color: #444;
        margin-bottom: 15px;
        font-size: 1rem;
      }
      .guide-box {
        display: inline-block;
        background: #eef4fa;
        color: var(--primary);
        padding: 8px 20px;
        border-radius: 20px;
        font-weight: 700;
        font-size: 0.9rem;
        margin-bottom: 25px;
      }
      .project-desc {
        background: #fff8e1; /* Light yellow for focus */
        padding: 15px;
        border-left: 4px solid var(--accent);
        text-align: left;
        font-size: 0.95rem;
        line-height: 1.5;
        color: #444;
        margin-bottom: 10px;
        border-radius: 0 8px 8px 0;
      }
      .project-desc strong {
        color: #d35400;
        display: block;
        margin-bottom: 4px;
        text-transform: uppercase;
        font-size: 0.85rem;
      }

      /* --- STREAM SECTION (Bottom) --- */
      .stream-wrapper {
        background: #000;
        padding: 0;
        position: relative;
        min-height: 240px;
        display: flex;
        justify-content: center;
        align-items: center;
        border-top: 1px solid #ddd;
        border-bottom: 1px solid #ddd;
      }
      img {
        width: 100%;
        max-width: 640px;
        display: block;
        transition: transform 0.4s ease;
      }
      
      /* --- CONTROLS --- */
      .controls {
        padding: 20px;
        background: #fdfdfd;
      }
      .btn-rotate {
        background: var(--primary);
        color: white;
        border: none;
        padding: 12px 25px;
        font-size: 15px;
        font-weight: 600;
        border-radius: 6px;
        cursor: pointer;
        display: inline-flex;
        align-items: center;
        gap: 8px;
        transition: background 0.2s, transform 0.1s;
      }
      .btn-rotate:hover { background: #002244; }
      .btn-rotate:active { transform: scale(0.98); }

    </style>
    <script>
      let isRotated = false;

      function rotateStream() {
        isRotated = !isRotated;
        const img = document.getElementById('cam-stream');
        img.style.transform = isRotated ? "rotate(180deg)" : "rotate(0deg)";
      }

      // Live Stream Logic
      window.onload = function() {
        const img = document.getElementById('cam-stream');
        
        function refreshImage() {
          const newImg = new Image();
          newImg.src = "/capture?t=" + new Date().getTime();
          
          newImg.onload = function() {
            img.src = this.src;
            refreshImage(); // Fetch next frame immediately
          };
          
          newImg.onerror = function() {
            setTimeout(refreshImage, 500); 
          };
        }
        refreshImage();
      };
    </script>
  </head>
  <body>
    <div class="container">
      
      <div class="header-section">
        <h1>Team-5</h1>
        <h2>MMD-1: Case Study</h2>
        
        <div class="course-info">
          Cooperative and Autonomous Systems (WS25/26)
        </div>
        
        <div class="guide-box">
          Guide: Prof. Dr. Igor Doric
        </div>

        <div class="project-desc">
          <strong>Campus Mail Delivery Robo</strong>
          Implementation of Cognitive Core for Quadrupedal Robot Using Path Planning and Dynamic Obstacle Avoidance.
        </div>
      </div>

      <div class="stream-wrapper">
        <img id='cam-stream' src='/capture' alt="Connecting to Camera Source...">
      </div>

      <div class="controls">
        <button class="btn-rotate" onclick="rotateStream()">
          <span>↻</span> Rotate Stream 180°
        </button>
      </div>

    </div>
  </body>
  </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

// ==== Setup ====
void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. Camera Init
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Use QVGA for streaming stability
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("❌ Camera init failed!");
    return;
  }

  // 2. WiFi AP Init (192.168.5.1)
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("AP Config Failed");
  }
  WiFi.softAP(ssid, password, 1, 0, 4);

  // 3. ESP-NOW Init
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed!");
    return;
  }
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 1; 
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // 4. Server Init
  server.on("/", HTTP_GET, handleRoot);
  server.on("/capture", HTTP_GET, handleCapture);
  server.begin();
  
  Serial.println("\n✅ System Ready");
  Serial.print("   Connect to: "); Serial.println(ssid);
  Serial.print("   Go to URL: http://"); Serial.println(WiFi.softAPIP());
}

// ==== Loop ====
void loop() {
  server.handleClient();
  delay(1); 
}