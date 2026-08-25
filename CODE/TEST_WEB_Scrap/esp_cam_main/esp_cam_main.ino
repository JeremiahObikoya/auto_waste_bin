/*
 * =============================================================================
 * AUTO WASTE BIN V2 — ESP32-CAM VISION NODE (MIDDLEMAN ARCHITECTURE)
 * =============================================================================
 * Board   : AI-Thinker ESP32-CAM
 * Role    : Captures JPEG frames and POSTs them via local Wi-Fi HTTP to a
 *           Python server on the laptop. Relays returned decisions via ESP-NOW
 *           to the Main ESP32 motor controller.
 * =============================================================================
 */

#include "esp_camera.h"
#include "secrets.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_now.h>

// ===================== AI-THINKER CAMERA PINOUT ==============================
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

// ===================== VISION STATES =========================================
enum VisionState {
  SCANNING       = 0,
  TRACKING       = 1,
  INTERACTION    = 2,
  RELEASE        = 3,
  SCAN_STEP_DONE = 4
};

VisionState visionState = SCANNING;
unsigned long lastQueryMs = 0;
unsigned long releaseStartMs = 0;
uint8_t receivedMode = 0; // 0=MANUAL, 1=AUTONOMOUS, 2=CAM_GUIDED, 3=PAUSED

// Query intervals (tuned for browser web scraping pipeline)
#define INTERVAL_STABILIZE_MS 4000 // 4 s stabilization after servo movement
#define INTERVAL_TRACK_MS     2500 // 2.5 s while tracking target
#define INTERVAL_INTER_MS     3000 // 3.0 s in drop-off interaction

// ===================== ESP-NOW PACKETS =======================================
typedef struct __attribute__((packed)) {
  uint8_t state;      // 0=SCANNING, 1=TRACKING, 2=INTERACTION, 3=RELEASE
  int8_t angleOffset; // ±30° offset from center
} CamPacket;

typedef struct __attribute__((packed)) {
  uint8_t systemMode; // 0=MANUAL, 1=AUTONOMOUS, 2=CAM_GUIDED, 3=PAUSED
} MainPacket;

CamPacket outPacket;
uint8_t mainMAC[6];

void onSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Optional transmit debug
}

void onReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data,
                int len) {
  if (len == sizeof(MainPacket)) {
    MainPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    receivedMode = pkt.systemMode;
    Serial.printf("[ESP-NOW RX] systemMode=%d — sending ACK\n", receivedMode);
    sendCamPacket(visionState, 0); // Send immediate ACK back to Main ESP32
  }
}

void sendCamPacket(uint8_t state, int8_t angleOff) {
  outPacket.state = state;
  outPacket.angleOffset = angleOff;
  esp_now_send(mainMAC, (uint8_t *)&outPacket, sizeof(outPacket));
}

// ===================== HTTP QUERY TO LAPTOP SERVER ===========================
String queryLaptopServer(uint8_t state, String &outDetection, int &outBboxX) {
  outDetection = "ERROR";
  outBboxX = 160;

  // Flush any queued DMA frames from previous angle to ensure 100% real-time frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);

  // Capture fresh live frame
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[Camera] Capture failed");
    return "";
  }

  String serverUrl = "http://" + String(LAPTOP_IP) + ":" + String(LAPTOP_PORT) +
                     "/process_frame?state=" + String(state);

  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "image/jpeg");
  http.setTimeout(120000); // 2-minute connection timeout (wait until Gemini finishes)

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

// ===================== SETUP ================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== AUTO WASTE BIN — ESP32-CAM (MIDDLEMAN MODE) ==="));

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

  // --- Hardware Sensor Inversion (Fix Upside Down Image) ---
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);   // 1 = Flip vertically (corrects upside down camera)
    s->set_hmirror(s, 1); // 1 = Mirror horizontally
  }
  Serial.println(F("✅ Camera initialized & image orientation corrected (QVGA 320x240)."));

  // --- 2. Wi-Fi Connection (STA Mode) ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to Wi-Fi '%s'...", WIFI_SSID);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ Wi-Fi connected! IP: %s (Channel: %d)\n",
                  WiFi.localIP().toString().c_str(), WiFi.channel());
  } else {
    Serial.println(F("\n⚠ Wi-Fi connection timed out. Check credentials."));
  }

  // --- 3. ESP-NOW Init ---
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("❌ ESP-NOW init failed"));
    while (true)
      delay(1000);
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceived);

  memcpy(mainMAC, MAIN_ESP32_MAC, 6);
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, mainMAC, 6);
  peer.channel = WiFi.channel();
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    Serial.printf("✅ ESP-NOW peer registered: %02X:%02X:%02X:%02X:%02X:%02X on Channel %d\n",
                  mainMAC[0], mainMAC[1], mainMAC[2], mainMAC[3], mainMAC[4],
                  mainMAC[5], peer.channel);
  } else {
    Serial.println(F("⚠ Peer add failed — check MAC address in secrets.h"));
  }
}

// ===================== LOOP =================================================
unsigned long lastPreviewMs = 0;

void sendPreviewFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (!fb)
    return;

  String serverUrl = "http://" + String(LAPTOP_IP) + ":" + String(LAPTOP_PORT) +
                     "/process_frame?preview=1";
  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "image/jpeg");
  http.setTimeout(1200);
  http.POST(fb->buf, fb->len);
  http.end();
  esp_camera_fb_return(fb);
}

void loop() {
  // Auto-reconnect to Wi-Fi if router/hotspot started late or dropped
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifiRetryMs = 0;
    if (millis() - lastWifiRetryMs >= 4000) {
      lastWifiRetryMs = millis();
      Serial.println(F("🔄 Connecting to Wi-Fi 'Traffic_ESP'..."));
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }

  // Send heartbeat every 1500ms so Main ESP32 Web Dashboard shows ESP32-CAM ONLINE
  static unsigned long lastHeartbeatMs = 0;
  if (millis() - lastHeartbeatMs >= 1500) {
    lastHeartbeatMs = millis();
    sendCamPacket(visionState, 0);
  }

  // When robot is not in CAM_GUIDED mode, stream preview frames every 600ms
  if (receivedMode != 2) {
    if (millis() - lastPreviewMs >= 600) {
      lastPreviewMs = millis();
      sendPreviewFrame();
    }
    delay(50);
    return;
  }

  unsigned long now = millis();

  switch (visionState) {

  // ── 0: SCANNING ────────────────────────────────────────────────────────
  case SCANNING: {
    // 1. Wait 4.0 seconds for camera servo to settle & view to stabilize
    if (now - lastQueryMs >= INTERVAL_STABILIZE_MS) {
      lastQueryMs = now;
      Serial.println(F("📷 [SCAN] Servo stabilized (4s). Capturing & sending frame to Gemini..."));
      String det = "ERROR";
      int bx = 160;
      queryLaptopServer(SCANNING, det, bx);

      if (det == "EXTENDED_ARM_WITH_OBJECT") {
        int8_t offset = (int8_t)((bx - 160) * 30 / 160);
        Serial.printf("🎯 [SCAN → TRACKING] bx=%d, offset=%d°\n", bx, offset);
        visionState = TRACKING;
        sendCamPacket(TRACKING, offset);
      } else if (det == "NONE") {
        // Confirmed complete NONE response -> trigger Main ESP32 to step camera servo to next angle immediately!
        Serial.println(F("⏭️ [SCAN] NONE confirmed. Advancing servo to next angle..."));
        sendCamPacket(SCAN_STEP_DONE, 0);
        lastQueryMs = millis(); // Reset 4s stabilization timer for next angle
      } else {
        // HTTP error, incomplete payload, or INVALID -> do NOT advance angle, retry!
        Serial.println(F("⚠️ [SCAN] Incomplete response / Network glitch. Retrying current angle..."));
        lastQueryMs = millis() - (INTERVAL_STABILIZE_MS - 1000); // Retry in 1 second
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
        Serial.println(F("[TRACKING → INTERACTION] Reached user."));
        trackMissCount = 0;
        visionState = INTERACTION;
        sendCamPacket(INTERACTION, 0);

      } else if (det == "TRACKING" || det == "EXTENDED_ARM_WITH_OBJECT") {
        trackMissCount = 0;
        int8_t off = (int8_t)((bx - 160) * 30 / 160);
        sendCamPacket(TRACKING, off);

      } else {
        trackMissCount++;
        if (trackMissCount >= 2) {
          Serial.println(F("[TRACKING → SCANNING] Target lost."));
          trackMissCount = 0;
          visionState = SCANNING;
          sendCamPacket(SCANNING, 0);
        } else {
          sendCamPacket(TRACKING, 0);
        }
      }
    }
    break;
  }

  // ── 2: INTERACTION ─────────────────────────────────────────────────────
  case INTERACTION: {
    sendCamPacket(INTERACTION, 0);

    if (now - lastQueryMs >= INTERVAL_INTER_MS) {
      lastQueryMs = now;
      String det;
      int bx;
      queryLaptopServer(INTERACTION, det, bx);

      if (det == "EMPTY_ARM" || det == "BACK_TURNED") {
        Serial.println(F("[INTERACTION → RELEASE] Drop confirmed."));
        visionState = RELEASE;
        releaseStartMs = now;
        sendCamPacket(RELEASE, 0);
      }
    }
    break;
  }

  // ── 3: RELEASE ─────────────────────────────────────────────────────────
  case RELEASE: {
    sendCamPacket(RELEASE, 0);
    if (now - releaseStartMs >= 3000) {
      visionState = SCANNING;
      Serial.println(F("[RELEASE → SCANNING] Resetting to scan."));
    }
    break;
  }
  }
}
