/*
 * =============================================================================
 * AUTO WASTE BIN V2 — ESP32-CAM VISION NODE
 * =============================================================================
 * Board   : AI-Thinker ESP32-CAM
 * Role    : 4-state vision machine. Captures frames, queries Gemini Vision API,
 *           sends [STATE, ANGLE_OFFSET] to Main ESP32 via ESP-NOW.
 *           Receives [SYSTEM_MODE] from Main ESP32 and suspends Gemini
 *           queries when the robot is not in CAM_GUIDED mode.
 *
 * Wiring (AI-Thinker ESP32-CAM):
 *   Camera     : OV2640 (all GPIOs used by camera module)
 *   NOTE       : There is NO servo on this board.
 *                The camera pan servo (GPIO 33) is on the MAIN ESP32.
 *
 * Required Libraries:
 *   (esp_camera, WiFi, esp_now, WiFiClientSecure are part of ESP32 Arduino core)
 *
 * ⚠️  Fill in YOUR credentials in secrets.h before flashing!
 * =============================================================================
 *
 * Vision States:
 *   0 SCANNING    – Gemini looks for "Extended Arm + Object" in the camera frame.
 *   1 TRACKING    – Robot curves toward target. Gemini monitors for STOP_PALM or PROXIMITY.
 *   2 INTERACTION – Robot halted. Gemini waits for waste drop confirmation.
 *   3 RELEASE     – Back-off commanded. Timer resets to SCANNING.
 *
 * ESP-NOW Packet Formats:
 *   CAM → Main : CamPacket  { uint8_t state; int8_t angleOffset; }
 *   Main → CAM : MainPacket { uint8_t systemMode; }
 *                systemMode: 0=MANUAL, 1=AUTONOMOUS_LEGACY, 2=CAM_GUIDED, 3=CAM_PAUSED
 * =============================================================================
 */

#include "secrets.h"          // ← Gitignored credentials
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_now.h>
#include "mbedtls/base64.h"

// ===================== AI-THINKER CAMERA PIN MAP ============================
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

// ===================== VISION STATE MACHINE =================================
enum VisionState : uint8_t {
  SCANNING     = 0,
  TRACKING     = 1,
  INTERACTION  = 2,
  RELEASE      = 3
};
VisionState visionState = SCANNING;

unsigned long releaseStartMs = 0;
#define RELEASE_DURATION_MS 4000  // ms to stay in RELEASE before resetting

// ===================== ESP-NOW PACKETS ======================================
/**
 * Sent from CAM → Main ESP32.
 * state       : current VisionState value (0–3)
 * angleOffset : person's angle from frame centre in degrees (negative=left, positive=right)
 */
struct CamPacket {
  uint8_t state;
  int8_t  angleOffset;
};

/**
 * Received from Main ESP32 → CAM.
 * systemMode  : 0=MANUAL, 1=AUTONOMOUS_LEGACY, 2=CAM_GUIDED
 */
struct MainPacket {
  uint8_t systemMode;
};

uint8_t mainMAC[] = MAIN_ESP32_MAC;
esp_now_peer_info_t peerInfo;

volatile uint8_t receivedMode = 0;  // 0 = MANUAL until we hear otherwise

CamPacket outPacket = {SCANNING, 0};

// ---- Callbacks (ESP32 Arduino core v3.x / IDF 5.x signatures) ----
void onSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Serial.printf("[ESP-NOW TX] %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(MainPacket)) {
    MainPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    receivedMode = pkt.systemMode;
    Serial.printf("[ESP-NOW RX] systemMode=%d\n", receivedMode);
  }
}

void sendCamPacket(uint8_t state, int8_t angleOff) {
  outPacket.state       = state;
  outPacket.angleOffset = angleOff;
  esp_now_send(mainMAC, (uint8_t *)&outPacket, sizeof(outPacket));
}

// ===================== GEMINI API ===========================================
#define GEMINI_HOST  "generativelanguage.googleapis.com"
#define GEMINI_MODEL "gemini-3.6-flash"

// Prompts — carefully worded for reliable JSON-only output
static const char PROMPT_SCAN[] =
  "You are the vision system of an autonomous waste bin robot. "
  "Analyse this camera frame and determine if a person has an EXTENDED ARM "
  "clearly HOLDING AN OBJECT (e.g. trash, a bottle, paper) directed toward "
  "the camera. Respond ONLY with raw JSON (no markdown, no explanation). "
  "If detected: {\"detection\":\"EXTENDED_ARM_WITH_OBJECT\",\"bbox_center_x\":PIXEL} "
  "where PIXEL (0-320) is the horizontal pixel of the arm/person centre. "
  "If not detected: {\"detection\":\"NONE\"}";

static const char PROMPT_TRACK[] =
  "You are the vision system of an autonomous waste bin robot in TRACKING mode. "
  "The robot is moving toward a person. Identify ONE of the following: "
  "STOP_PALM - person raises an open empty palm (stop gesture); "
  "PROXIMITY - person fills more than 65% of frame width (robot is very close); "
  "TRACKING - person is visible with arm extended holding object. "
  "Respond ONLY with raw JSON. "
  "Format: {\"detection\":\"STOP_PALM\"} "
  "or {\"detection\":\"PROXIMITY\"} "
  "or {\"detection\":\"TRACKING\",\"bbox_center_x\":PIXEL}";

static const char PROMPT_INTERACT[] =
  "You are the vision system of an autonomous waste bin robot in drop-off mode. "
  "The robot is stationary waiting for user to deposit waste. "
  "Identify ONE of the following: "
  "EMPTY_ARM - person's arm is extended but clearly empty (waste dropped); "
  "BACK_TURNED - person turned back and is walking away; "
  "WAITING - person still holding waste. "
  "Respond ONLY with raw JSON: "
  "{\"detection\":\"EMPTY_ARM\"} or {\"detection\":\"BACK_TURNED\"} or {\"detection\":\"WAITING\"}";

// Rate-limited Gemini query intervals — tuned for the free tier (15 RPM / 1500 RPD)
//   SCANNING     : 6 s  → 10 RPM  (no confirmed target yet, relax the rate)
//   TRACKING     : 4 s  → 15 RPM  (max safe rate while actively following a target)
//   INTERACTION  : 5 s  → 12 RPM  (waiting for drop — infrequent checks are fine)
unsigned long lastGeminiMs = 0;
#define GEMINI_INTERVAL_SCAN_MS    6000
#define GEMINI_INTERVAL_TRACK_MS   4000
#define GEMINI_INTERVAL_INTER_MS   5000

/**
 * @brief Capture a JPEG frame, base64-encode it, POST to Gemini Vision API,
 *        and return the raw text the model produces.
 */
String queryGemini(const char *prompt) {
  // --- Capture frame ---
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[Gemini] Frame capture failed");
    return "{\"detection\":\"NONE\"}";
  }

  // --- Base64 encode (use PSRAM if available) ---
  size_t b64MaxLen = ((fb->len + 2) / 3) * 4 + 1;
  uint8_t *b64Buf = (uint8_t *)ps_malloc(b64MaxLen);
  if (!b64Buf) b64Buf = (uint8_t *)malloc(b64MaxLen);
  if (!b64Buf) {
    esp_camera_fb_return(fb);
    Serial.println("[Gemini] malloc failed");
    return "{\"detection\":\"NONE\"}";
  }

  size_t b64Len = 0;
  mbedtls_base64_encode(b64Buf, b64MaxLen, &b64Len, fb->buf, fb->len);
  b64Buf[b64Len] = '\0';
  esp_camera_fb_return(fb);   // Return frame buffer immediately after encode

  // --- Build HTTP body prefix and suffix (prompt is JSON-escaped) ---
  String escapedPrompt = "";
  for (int i = 0; prompt[i] != '\0'; i++) {
    if (prompt[i] == '"')       escapedPrompt += "\\\"";
    else if (prompt[i] == '\\')  escapedPrompt += "\\\\";
    else if (prompt[i] == '\n')  escapedPrompt += "\\n";
    else if (prompt[i] == '\r')  continue;
    else                         escapedPrompt += prompt[i];
  }

  String prefix = "{\"contents\":[{\"parts\":[{\"text\":\"";
  prefix += escapedPrompt;
  prefix += "\"},{\"inline_data\":{\"mime_type\":\"image/jpeg\",\"data\":\"";

  String suffix = "\"}}]}]}";
  size_t totalBodyLen = prefix.length() + b64Len + suffix.length();

  // --- HTTPS connection ---
  String endpoint = String("/v1beta/models/") + GEMINI_MODEL +
                    ":generateContent?key=" + GEMINI_API_KEY;

  WiFiClientSecure client;
  client.setInsecure();   // Skip cert verification (acceptable for prototype)
  client.setTimeout(12);

  if (!client.connect(GEMINI_HOST, 443)) {
    free(b64Buf);
    Serial.println("[Gemini] Connection failed");
    return "{\"detection\":\"NONE\"}";
  }

  // --- Send request with chunked body to avoid giant String ---
  client.printf("POST %s HTTP/1.1\r\n", endpoint.c_str());
  client.printf("Host: %s\r\n", GEMINI_HOST);
  client.println("Content-Type: application/json");
  client.printf("Content-Length: %u\r\n", (unsigned)totalBodyLen);
  client.println("Connection: close");
  client.println();
  client.print(prefix);
  client.write(b64Buf, b64Len);
  client.print(suffix);
  free(b64Buf);

  // --- Read response, skip headers ---
  String responseBody = "";
  unsigned long deadline = millis() + 10000;
  unsigned long lastByteMs = millis();
  bool pastHeaders = false;

  while ((client.connected() || client.available()) && millis() < deadline) {
    if (client.available()) {
      if (!pastHeaders) {
        String line = client.readStringUntil('\n');
        if (line == "\r" || line.length() == 0) {
          pastHeaders = true;
          lastByteMs  = millis();
        }
      } else {
        responseBody += client.readString();
        lastByteMs = millis();
        // Exit early once complete text field is parsed
        if (responseBody.indexOf("\"text\":") != -1 && responseBody.indexOf("\"finishReason\"") != -1) {
          break;
        }
      }
    } else {
      if (pastHeaders && responseBody.length() > 0 && (millis() - lastByteMs > 400)) {
        break; // Full response body received
      }
      delay(15);
    }
  }
  client.stop();

  // --- Extract the "text" field from the Gemini JSON response ---
  // Gemini returns: {"candidates":[{"content":{"parts":[{"text":"..."}]}}]}
  int tIdx = responseBody.indexOf("\"text\":");
  if (tIdx == -1) {
    Serial.println("[Gemini] No 'text' field found");
    Serial.println(responseBody.substring(0, 300));
    return "{\"detection\":\"NONE\"}";
  }

  int qStart = responseBody.indexOf('"', tIdx + 7);
  int qEnd   = qStart + 1;
  // Walk forward, skipping escaped quotes
  while (qEnd < (int)responseBody.length()) {
    if (responseBody[qEnd] == '"' && responseBody[qEnd - 1] != '\\') break;
    qEnd++;
  }

  String raw = responseBody.substring(qStart + 1, qEnd);
  // Unescape JSON string escapes
  raw.replace("\\\"", "\"");
  raw.replace("\\\\", "\\");
  raw.replace("\\n",  " ");

  Serial.printf("[Gemini] → %s\n", raw.c_str());
  return raw;
}

/** Extract the "detection" value from a Gemini JSON response string. */
String parseDetection(const String &json) {
  int idx = json.indexOf("\"detection\"");
  if (idx == -1) {
    // Fallback keyword scanning in case model formatting varies slightly
    if (json.indexOf("EXTENDED_ARM_WITH_OBJECT") != -1) return "EXTENDED_ARM_WITH_OBJECT";
    if (json.indexOf("STOP_PALM") != -1)                return "STOP_PALM";
    if (json.indexOf("PROXIMITY") != -1)                return "PROXIMITY";
    if (json.indexOf("EMPTY_ARM") != -1)                return "EMPTY_ARM";
    if (json.indexOf("BACK_TURNED") != -1)              return "BACK_TURNED";
    if (json.indexOf("TRACKING") != -1)                 return "TRACKING";
    return "NONE";
  }
  int colon = json.indexOf(':', idx);
  if (colon == -1) return "NONE";
  int q1 = json.indexOf('"', colon);
  if (q1 == -1) return "NONE";
  int q2 = json.indexOf('"', q1 + 1);
  if (q2 == -1) return "NONE";
  return json.substring(q1 + 1, q2);
}

/** Extract bbox_center_x (pixel); returns 160 (centre) if missing. */
int parseBboxX(const String &json) {
  int idx = json.indexOf("\"bbox_center_x\"");
  if (idx == -1) return 160;
  int colon = json.indexOf(':', idx);
  if (colon == -1) return 160;
  int val = json.substring(colon + 1).toInt();
  if (val <= 0 || val > 320) return 160;
  return val;
}

// ===================== SETUP ================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n========== AUTO WASTE BIN — ESP32-CAM VISION NODE =========="));

  // --- 1. Camera initialisation (OV2640 AI-Thinker config) ---
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = Y2_GPIO_NUM;
  cfg.pin_d1       = Y3_GPIO_NUM;
  cfg.pin_d2       = Y4_GPIO_NUM;
  cfg.pin_d3       = Y5_GPIO_NUM;
  cfg.pin_d4       = Y6_GPIO_NUM;
  cfg.pin_d5       = Y7_GPIO_NUM;
  cfg.pin_d6       = Y8_GPIO_NUM;
  cfg.pin_d7       = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = FRAMESIZE_QVGA;   // 320 × 240 — good balance of speed & detail
  cfg.jpeg_quality = 12;               // 0 (best) – 63 (worst), 12 is fine for vision
  cfg.fb_count     = 1;

  esp_err_t camErr = esp_camera_init(&cfg);
  if (camErr != ESP_OK) {
    Serial.printf("❌ Camera init FAILED: 0x%x\n", camErr);
    while (true) delay(1000);
  }
  Serial.println(F("✅ Camera init OK (QVGA 320×240 JPEG)"));

  // --- 2. Connect to Wi-Fi (STA mode — needed for Gemini HTTPS) ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("🌐 Connecting to Wi-Fi '%s'", WIFI_SSID);
  unsigned long wStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wStart < 20000) {
    delay(400);
    Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ Wi-Fi connected! IP: %s  Ch: %d\n",
                  WiFi.localIP().toString().c_str(), WiFi.channel());
  } else {
    Serial.println(F("\n⚠️  Wi-Fi timeout — Gemini calls will fail until connected."));
  }

  // --- 4. ESP-NOW init (works alongside STA Wi-Fi) ---
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("❌ ESP-NOW init FAILED"));
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceived);

  // Register Main ESP32 as peer
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, mainMAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;  // Match SoftAP channel on Main ESP32
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println(F("✅ ESP-NOW peer (Main ESP32) registered"));
  } else {
    Serial.println(F("⚠️  ESP-NOW peer add FAILED — check MAIN_ESP32_MAC in secrets.h"));
  }

  lastGeminiMs = millis();
  Serial.println(F("🚀 Vision Node ready — entering SCANNING state\n"));
}

// ===================== MAIN LOOP ============================================
void loop() {
  unsigned long now = millis();

  // ── IDLE when Main ESP32 is NOT in CAM_GUIDED mode ────────────────────────
  // systemMode: 0=MANUAL, 1=AUTONOMOUS_LEGACY, 2=CAM_GUIDED, 3=CAM_PAUSED
  // Only run Gemini when systemMode == 2
  if (receivedMode != 2) {
    static unsigned long lastIdleMs = 0;
    if (now - lastIdleMs >= 2000) {
      lastIdleMs = now;
      sendCamPacket(SCANNING, 0);
      Serial.printf("[Vision] Mode=%d — Gemini suspended.\n", receivedMode);
    }
    // Reset state machine so it starts fresh when CAM_GUIDED is activated
    if (visionState != SCANNING) visionState = SCANNING;
    return;
  }

  // ── CAM_GUIDED mode — run 4-state machine ─────────────────────────────────

  switch (visionState) {

    // ── State 0: SCANNING ──────────────────────────────────────────────────
    case SCANNING: {
      sendCamPacket(SCANNING, 0);   // Tell main: still scanning

      // Rate-limited Gemini query — 6 s interval → 10 RPM (well under 15 RPM cap)
      if (now - lastGeminiMs >= GEMINI_INTERVAL_SCAN_MS) {
        lastGeminiMs    = now;
        String json     = queryGemini(PROMPT_SCAN);
        String det      = parseDetection(json);

        if (det == "EXTENDED_ARM_WITH_OBJECT") {
          int    bx     = parseBboxX(json);
          // Map pixel offset to degrees: frame is 320px wide, ±30° max
          int8_t offset = (int8_t)((bx - 160) * 30 / 160);
          Serial.printf("[SCANNING → TRACKING]  bx=%d  offset=%d°\n", bx, offset);
          visionState = TRACKING;
          sendCamPacket(TRACKING, offset);
        }
      }
      break;
    }

    // ── State 1: TRACKING ──────────────────────────────────────────────────
    case TRACKING: {
      static int trackMissCount = 0;

      // Rate-limited Gemini query — 4 s interval → 15 RPM (max safe free-tier rate)
      if (now - lastGeminiMs >= GEMINI_INTERVAL_TRACK_MS) {
        lastGeminiMs = now;
        String json  = queryGemini(PROMPT_TRACK);
        String det   = parseDetection(json);

        if (det == "STOP_PALM") {
          // User cancels — skip drop-off, go straight to back-off
          Serial.println(F("[TRACKING → RELEASE]  Stop palm!"));
          trackMissCount = 0;
          visionState    = RELEASE;
          releaseStartMs = now;
          sendCamPacket(RELEASE, 0);

        } else if (det == "PROXIMITY") {
          // Robot is close enough — halt and wait for drop
          Serial.println(F("[TRACKING → INTERACTION]  Target reached."));
          trackMissCount = 0;
          visionState    = INTERACTION;
          sendCamPacket(INTERACTION, 0);

        } else if (det == "TRACKING" || det == "EXTENDED_ARM_WITH_OBJECT") {
          trackMissCount = 0;
          int    bx  = parseBboxX(json);
          int8_t off = (int8_t)((bx - 160) * 30 / 160);
          sendCamPacket(TRACKING, off);
          Serial.printf("[TRACKING]  bx=%d  off=%d°\n", bx, off);

        } else {
          trackMissCount++;
          if (trackMissCount >= 2) {
            // Target lost after 2 consecutive misses — return to scan
            Serial.println(F("[TRACKING → SCANNING]  Target lost."));
            trackMissCount = 0;
            visionState    = SCANNING;
            sendCamPacket(SCANNING, 0);
          } else {
            Serial.println(F("[TRACKING]  Target momentarily unclear — holding tracking..."));
            sendCamPacket(TRACKING, 0);
          }
        }
      }
      break;
    }

    // ── State 2: INTERACTION ───────────────────────────────────────────────
    case INTERACTION: {
      sendCamPacket(INTERACTION, 0);  // Keep Main halted

      // Rate-limited Gemini query — 5 s interval → 12 RPM
      if (now - lastGeminiMs >= GEMINI_INTERVAL_INTER_MS) {
        lastGeminiMs = now;
        String json  = queryGemini(PROMPT_INTERACT);
        String det   = parseDetection(json);

        if (det == "EMPTY_ARM" || det == "BACK_TURNED") {
          Serial.printf("[INTERACTION → RELEASE]  Drop confirmed: %s\n", det.c_str());
          visionState    = RELEASE;
          releaseStartMs = now;
          sendCamPacket(RELEASE, 0);
        }
        // If "WAITING", loop and keep halting
      }
      break;
    }

    // ── State 3: RELEASE ───────────────────────────────────────────────────
    case RELEASE: {
      sendCamPacket(RELEASE, 0);   // Main executes 2s reverse + 180° turn

      if (now - releaseStartMs >= RELEASE_DURATION_MS) {
        Serial.println(F("[RELEASE → SCANNING]  Reset. Ready for next user.\n"));
        visionState = SCANNING;
      }
      break;
    }
  }
}
