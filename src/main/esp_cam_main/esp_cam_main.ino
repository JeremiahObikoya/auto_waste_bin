/*
 * =============================================================================
 * AUTO WASTE BIN V2 — ESP32-CAM VISION NODE (WI-FI UDP + GEMINI STREAMING)
 * =============================================================================
 * Board   : AI Thinker ESP32-CAM
 * Role    : Captures live QVGA frames, runs vision state machine, queries the
 *           laptop Python Flask server (with Gemini AI), and transmits state
 *           updates over Wi-Fi UDP broadcast to the Main ESP32 navigation node.
 *
 * Camera Pan Servo: Connected to GPIO 33 on the MAIN ESP32 (controlled via
 * UDP).
 *
 * Network Configuration:
 *   - Wi-Fi Router : Joins Traffic_ESP (or configured SSID)
 *   - UDP Broadcast: Listens on port 8889, broadcasts to port 8888
 * (255.255.255.255)
 *   - Laptop Server: Sends HTTP POST to http://<laptop_ip>:5000/process_frame
 *
 * Flash Frequency: 80MHz | Flash Mode: QIO | Partition Scheme: Huge APP (3MB)
 * =============================================================================
 */

#include "esp_camera.h"
#include "secrets.h" // ← Router credentials & UDP ports
#include <Arduino.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>

Preferences camPrefs;

// ===================== AI THINKER CAMERA PINOUT ==============================
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ===================== SERVER / CONFIGURATION ================================
char laptopIp[32] = LAPTOP_IP;
#define LAPTOP_PORT 5000
char wifiSsid[33] = WIFI_SSID;
char wifiPass[65] = WIFI_PASSWORD;
bool newWifiPending = false;

// ===================== VISION STATE MACHINE ==================================
enum VisionState : uint8_t {
  SCANNING = 0,
  TRACKING = 1,
  INTERACTION = 2,
  RELEASE = 3,
  SCAN_STEP_DONE = 4,
  OBSTACLE_CHECK = 5,
  IS_TARGET = 6,
  IS_OBSTACLE = 7
};

VisionState visionState = SCANNING;
unsigned long lastQueryMs = 0;
unsigned long releaseStartMs = 0;
uint8_t receivedMode =
    0; // 0=MANUAL, 1=AUTONOMOUS, 2=CAM_GUIDED, 3=PAUSED, 5=OBSTACLE_CHECK

// Query intervals
#define INTERVAL_STABILIZE_MS 4000 // 4 s stabilization after servo movement
#define INTERVAL_TRACK_MS 2500     // 2.5 s while tracking target
#define INTERVAL_INTER_MS 3000     // 3.0 s in drop-off interaction

// ===================== UDP PACKETS (WI-FI ROUTER) ============================
typedef struct __attribute__((packed)) {
  uint8_t
      state; // 0=SCANNING, 1=TRACKING, 2=INTERACTION, 3=RELEASE,
             // 4=SCAN_STEP_DONE, 5=OBSTACLE_CHECK, 6=IS_TARGET, 7=IS_OBSTACLE
  int8_t angleOffset; // ±30° offset from center
} CamPacket;

typedef struct __attribute__((packed)) {
  uint8_t pktType; // 0=Mode/Heartbeat, 1=Config Sync, 5=Trigger Obstacle Check
  uint8_t systemMode; // 0=MANUAL, 1=AUTONOMOUS, 2=CAM_GUIDED, 3=PAUSED,
                      // 5=OBSTACLE_CHECK
  char laptopIp[32];
  char wifiSsid[33];
  char wifiPass[65];
} MainPacket;

CamPacket outPacket;
WiFiUDP udp;

void sendCamPacket(uint8_t st, int8_t angleOff) {
  outPacket.state = st;
  outPacket.angleOffset = angleOff;

  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket("255.255.255.255", UDP_MAIN_RX_PORT);
    udp.write((const uint8_t *)&outPacket, sizeof(outPacket));
    udp.endPacket();

    // For critical step transitions, send a 2-packet burst to guarantee
    // delivery
    if (st == SCAN_STEP_DONE || st == IS_TARGET || st == IS_OBSTACLE ||
        st == TRACKING) {
      delay(5);
      udp.beginPacket("255.255.255.255", UDP_MAIN_RX_PORT);
      udp.write((const uint8_t *)&outPacket, sizeof(outPacket));
      udp.endPacket();
    }
  }
}

// ===================== HTTP QUERY TO LAPTOP SERVER ===========================
String queryLaptopServer(uint8_t state, String &outDetection, int &outBboxX) {
  outDetection = "ERROR";
  outBboxX = 160;

  // Flush queued DMA frames from previous angle to ensure 100% fresh live frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb)
    esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (fb)
    esp_camera_fb_return(fb);

  // Capture fresh live frame
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println(F("❌ [Camera] Capture failed"));
    return "";
  }

  String serverUrl = "http://" + String(laptopIp) + ":" + String(LAPTOP_PORT) +
                     "/process_frame?state=" + String(state);

  HTTPClient http;
  http.setReuse(false);
  http.begin(serverUrl);
  http.addHeader("Content-Type", "image/jpeg");
  http.setTimeout(45000); // 45-second connection timeout

  unsigned long t0 = millis();
  int httpCode = http.POST(fb->buf, fb->len);
  unsigned long duration = millis() - t0;

  esp_camera_fb_return(fb);

  if (httpCode == 200) {
    String payload = http.getString();
    http.end();

    // Parse detection
    int detIdx = payload.indexOf("\"detection\"");
    if (detIdx != -1) {
      int colon = payload.indexOf(':', detIdx);
      int q1 = payload.indexOf('"', colon);
      int q2 = payload.indexOf('"', q1 + 1);
      if (q1 != -1 && q2 != -1) {
        outDetection = payload.substring(q1 + 1, q2);
      }
    }

    // Parse bbox_center_x
    int bxIdx = payload.indexOf("\"bbox_center_x\"");
    if (bxIdx != -1) {
      int colon = payload.indexOf(':', bxIdx);
      if (colon != -1) {
        int val = payload.substring(colon + 1).toInt();
        if (val > 0 && val <= 320)
          outBboxX = val;
      }
    }

    Serial.printf("[Laptop %lums] → %s (bx=%d)\n", duration,
                  outDetection.c_str(), outBboxX);
    return payload;
  } else {
    Serial.printf("[Laptop] HTTP error %d (%lums)\n", httpCode, duration);
    http.end();
    outDetection = "ERROR";
    return "";
  }
}

void handleObstacleCheck() {
  Serial.println(F(
      "🚨 [OBSTACLE CHECK] Capturing frame to verify obstacle with Gemini..."));
  String det = "ERROR";
  int bx = 160;
  queryLaptopServer(OBSTACLE_CHECK, det, bx);

  if (det == "IS_TARGET" || det == "PROXIMITY" ||
      det == "EXTENDED_ARM_WITH_OBJECT" || det == "STOP_PALM") {
    Serial.println(F("🎯 [OBSTACLE CHECK] Gemini confirmed: IS_TARGET!"));
    sendCamPacket(IS_TARGET, 0);
  } else {
    Serial.println(
        F("🚧 [OBSTACLE CHECK] Gemini confirmed: IS_OBSTACLE (or none)."));
    sendCamPacket(IS_OBSTACLE, 0);
  }
}

// ===================== SETUP ================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== AUTO WASTE BIN — ESP32-CAM (UDP WI-FI MODE) ==="));

  // --- 1. Camera Init ---
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;
  cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM;
  cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM;
  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;
  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM;
  cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM;
  cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn = PWDN_GPIO_NUM;
  cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size = FRAMESIZE_QVGA; // 320x240
  cfg.jpeg_quality = 12;
  cfg.fb_count = 2;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed: 0x%x\n", err);
    while (true)
      delay(1000);
  }
  Serial.println(F("✅ Camera initialized (320x240 QVGA)"));

  // Adjust sensor settings
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_vflip(s,
                 1); // 1 = Flip vertically (corrects upside down camera mount)
    s->set_hmirror(s, 1); // 1 = Mirror horizontally (maintains correct
                          // left/right orientation)
  }

  // --- 2. Load Persisted Config from Flash NVS ---
  camPrefs.begin("cam_cfg", false);
  if (camPrefs.isKey("laptop_ip")) {
    camPrefs.getString("laptop_ip", laptopIp, sizeof(laptopIp));
    Serial.printf("📦 Loaded persisted Laptop IP from NVS: %s\n", laptopIp);
  }
  if (camPrefs.isKey("wifi_ssid")) {
    camPrefs.getString("wifi_ssid", wifiSsid, sizeof(wifiSsid));
    camPrefs.getString("wifi_pass", wifiPass, sizeof(wifiPass));
    Serial.printf("📦 Loaded persisted Wi-Fi from NVS: SSID='%s'\n", wifiSsid);
  }
  camPrefs.end();

  // --- 3. Connect to Wi-Fi Router (STA Mode) ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPass);
  Serial.printf("Connecting to Wi-Fi '%s'...", wifiSsid);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 25) {
    delay(400);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ Wi-Fi connected! IP: %s (Channel: %d)\n",
                  WiFi.localIP().toString().c_str(), WiFi.channel());
  } else {
    Serial.println(F("\n⚠ Wi-Fi connection timed out. Check credentials."));
  }

  // --- 4. UDP Init ---
  udp.begin(UDP_CAM_RX_PORT);
  Serial.printf("✅ UDP listening on port %d (Target: %d)\n", UDP_CAM_RX_PORT,
                UDP_MAIN_RX_PORT);
}

// ===================== LOOP =================================================
unsigned long lastPreviewMs = 0;

void sendPreviewFrame() {
  if (laptopIp[0] == '\0' || receivedMode != 0)
    return; // Only preview in manual mode

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb)
    esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (!fb)
    return;

  String serverUrl = "http://" + String(laptopIp) + ":" + String(LAPTOP_PORT) +
                     "/process_frame?preview=1";

  HTTPClient http;
  http.setReuse(false);
  http.begin(serverUrl);
  http.addHeader("Content-Type", "image/jpeg");
  http.setTimeout(800); // 800ms fast timeout
  http.POST(fb->buf, fb->len);
  http.end();
  esp_camera_fb_return(fb);
}

void loop() {
  // Check incoming UDP packets from Main ESP32
  int pSize = udp.parsePacket();
  if (pSize >= (int)sizeof(MainPacket)) {
    MainPacket pkt;
    udp.read((uint8_t *)&pkt, sizeof(pkt));
    receivedMode = pkt.systemMode;
    Serial.printf("[UDP RX] systemMode=%d, pktType=%d\n", pkt.systemMode,
                  pkt.pktType);

    // Handle trigger for obstacle check from Main ESP32
    if (pkt.pktType == 5 || pkt.systemMode == 5) {
      handleObstacleCheck();
      return;
    }

    // Handle explicit vision reset command from Main ESP32 (e.g. after 90° turn
    // / departure / drop-off)
    if (pkt.pktType == 4) {
      Serial.println(
          F("🔄 [UDP RX] Main ESP32 commanded fresh vision scan -> Flushing "
            "DMA buffers & resetting visionState = SCANNING"));
      visionState = SCANNING;
      // Flush DMA buffers to guarantee subsequent capture is 100% fresh from
      // current heading
      for (int i = 0; i < 3; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb)
          esp_camera_fb_return(fb);
      }
      lastQueryMs =
          millis() - (INTERVAL_STABILIZE_MS -
                      800); // Wait 800ms for chassis settling then capture
      return;
    }

    // Process dynamic config sync from Main ESP32 web interface
    if (pkt.pktType == 1 || pkt.laptopIp[0] != '\0') {
      if (strlen(pkt.laptopIp) >= 7 &&
          strncmp(laptopIp, pkt.laptopIp, sizeof(laptopIp)) != 0) {
        strncpy(laptopIp, pkt.laptopIp, sizeof(laptopIp));
        camPrefs.begin("cam_cfg", false);
        camPrefs.putString("laptop_ip", laptopIp);
        camPrefs.end();
        Serial.printf("📡 [NVS] Laptop IP updated from Main ESP32: %s\n",
                      laptopIp);
      }
      if (strlen(pkt.wifiSsid) > 0 &&
          strncmp(wifiSsid, pkt.wifiSsid, sizeof(wifiSsid)) != 0) {
        strncpy(wifiSsid, pkt.wifiSsid, sizeof(wifiSsid));
        strncpy(wifiPass, pkt.wifiPass, sizeof(wifiPass));
        camPrefs.begin("cam_cfg", false);
        camPrefs.putString("wifi_ssid", wifiSsid);
        camPrefs.putString("wifi_pass", wifiPass);
        camPrefs.end();
        newWifiPending = true;
        Serial.printf("📡 [NVS] Wi-Fi SSID updated from Main ESP32: '%s'\n",
                      wifiSsid);
      }
    }
  }

  // Handle Wi-Fi credentials change from Main ESP32
  if (newWifiPending) {
    newWifiPending = false;
    Serial.printf("🔄 Connecting to new Wi-Fi '%s'...\n", wifiSsid);
    WiFi.disconnect();
    WiFi.begin(wifiSsid, wifiPass);
  }

  // Auto-reconnect to Wi-Fi if router dropped
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifiRetryMs = 0;
    if (millis() - lastWifiRetryMs >= 5000) {
      lastWifiRetryMs = millis();
      Serial.printf("🔄 Reconnecting to Wi-Fi '%s'...\n", wifiSsid);
      WiFi.disconnect();
      WiFi.begin(wifiSsid, wifiPass);
    }
  }

  // Send dedicated heartbeat every 2000ms so Main ESP32 Web Dashboard shows
  // ESP32-CAM ONLINE (Always send state=0 so stale decisions like
  // SCAN_STEP_DONE are never re-triggered!)
  static unsigned long lastHeartbeatMs = 0;
  if (millis() - lastHeartbeatMs >= 2000) {
    lastHeartbeatMs = millis();
    sendCamPacket(SCANNING, 0);
  }

  // Gentle throttled preview (every 4s) only in MANUAL mode (mode 0)
  if (receivedMode == 0) {
    if (millis() - lastPreviewMs >= 4000) {
      lastPreviewMs = millis();
      sendPreviewFrame();
    }
  }

  // If robot is not in CAM_GUIDED mode, do not run vision query machine
  if (receivedMode != 2) {
    delay(20);
    return;
  }

  unsigned long now = millis();

  switch (visionState) {

  // ── 0: SCANNING ────────────────────────────────────────────────────────
  case SCANNING: {
    // 1. Wait 4.0 seconds for camera servo to settle & view to stabilize
    if (now - lastQueryMs >= INTERVAL_STABILIZE_MS) {
      lastQueryMs = now;
      Serial.println(F("📷 [SCAN] Servo stabilized (4s). Capturing & sending "
                       "frame to Gemini..."));
      String det = "ERROR";
      int bx = 160;
      queryLaptopServer(SCANNING, det, bx);

      if (det == "EXTENDED_ARM_WITH_OBJECT") {
        int8_t offset = (int8_t)((bx - 160) * 30 / 160);
        Serial.printf("🎯 [SCAN → TRACKING] bx=%d, offset=%d°\n", bx, offset);
        visionState = TRACKING;
        sendCamPacket(TRACKING, offset);
      } else if (det == "NONE") {
        // Confirmed complete NONE response -> trigger Main ESP32 to step camera
        // servo to next angle immediately!
        Serial.println(
            F("⏭️ [SCAN] NONE confirmed. Advancing servo to next angle..."));
        sendCamPacket(SCAN_STEP_DONE, 0);
        lastQueryMs = millis(); // Reset 4s stabilization timer for next angle
      } else {
        // HTTP error, incomplete payload, or INVALID -> do NOT advance angle,
        // retry!
        Serial.println(F("⚠️ [SCAN] Incomplete response / Network glitch. "
                         "Retrying current angle..."));
        lastQueryMs =
            millis() - (INTERVAL_STABILIZE_MS - 1000); // Retry in 1 second
      }
    }
    break;
  }

  // ── 1: TRACKING ────────────────────────────────────────────────────────
  case TRACKING: {
    static int trackMissCount = 0;

    if (now - lastQueryMs >= INTERVAL_TRACK_MS) {
      lastQueryMs = now;
      String det;
      int bx;
      queryLaptopServer(TRACKING, det, bx);

      if (det == "STOP_PALM") {
        Serial.println(F("[TRACKING → RELEASE] Stop palm received."));
        trackMissCount = 0;
        visionState = RELEASE;
        releaseStartMs = now;
        sendCamPacket(RELEASE, 0);
      } else if (det == "PROXIMITY") {
        Serial.println(
            F("[TRACKING → INTERACTION] Close proximity confirmed."));
        trackMissCount = 0;
        visionState = INTERACTION;
        sendCamPacket(INTERACTION, 0);
      } else if (det == "EXTENDED_ARM_WITH_OBJECT" || det == "TRACKING") {
        trackMissCount = 0;
        int8_t offset = (int8_t)((bx - 160) * 30 / 160);
        Serial.printf("🎯 [TRACKING ACTIVE] bx=%d, offset=%d°\n", bx, offset);
        sendCamPacket(TRACKING, offset);
      } else if (det == "NONE") {
        trackMissCount++;
        Serial.printf("[TRACKING] Target not seen (miss %d/3)\n",
                      trackMissCount);
        if (trackMissCount >= 3) {
          Serial.println(
              F("[TRACKING → SCANNING] Lost target. Restarting scan."));
          trackMissCount = 0;
          visionState = SCANNING;
          sendCamPacket(SCANNING, 0);
        }
      }
    }
    break;
  }

  // ── 2: INTERACTION ─────────────────────────────────────────────────────
  case INTERACTION: {
    if (now - lastQueryMs >= INTERVAL_INTER_MS) {
      lastQueryMs = now;
      String det;
      int bx;
      queryLaptopServer(INTERACTION, det, bx);

      if (det == "STOP_PALM" || det == "RELEASE") {
        Serial.println(F("[INTERACTION → RELEASE] User done / showing palm."));
        visionState = RELEASE;
        releaseStartMs = now;
        sendCamPacket(RELEASE, 0);
      }
    }
    break;
  }

  // ── 3: RELEASE ─────────────────────────────────────────────────────────
  case RELEASE: {
    if (now - releaseStartMs >= 3000) {
      Serial.println(
          F("[RELEASE → SCANNING] Release complete. Restarting scan."));
      visionState = SCANNING;
      sendCamPacket(SCANNING, 0);
    }
    break;
  }

  default:
    visionState = SCANNING;
    break;
  }
}
