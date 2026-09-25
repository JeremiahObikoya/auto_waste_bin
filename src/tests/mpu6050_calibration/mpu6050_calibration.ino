/*
 * =============================================================================
 * AUTO WASTE BIN — MPU6050 WEB CALIBRATION & LIVE MONITOR
 * =============================================================================
 * Board   : ESP32 Dev Module
 * Pins    : I2C SDA = GPIO 21, SCL = GPIO 22
 * Network : SoftAP (Connect to "Auto_Waste_Bin_AP" / pass: "12345678")
 * URL     : http://192.168.4.1
 * 
 * Features:
 *  1. Non-blocking SoftAP & Web Dashboard (always accessible even if MPU6050 is disconnected).
 *  2. Continuous background auto-reconnection and I2C bus recovery until MPU6050 is found.
 *  3. Live Raw & Physical MPU6050 data streaming (Accel, Gyro, Temperature).
 *  4. One-click Web Calibration for Hardware Register Offsets & Software Offsets.
 *  5. Instant copy-paste code snippets for both hardware & software configurations.
 * =============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ======================== PIN & AP DEFINITIONS ===============================
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define MPU_ADDR    0x68

const char *AP_SSID = "Auto_Waste_Bin_AP";
const char *AP_PASS = "12345678";

WebServer server(80);
Adafruit_MPU6050 mpu;
bool mpuConnected = false;
unsigned long lastReconnectAttempt = 0;
int mpuFailCount = 0;

// ======================== SENSOR CONFIGURATION ===============================
// Recommended settings for differential-drive ground robot:
// - Accel: ±2g (16384 LSB/g) -> Maximum resolution for floor tilt and level detection
// - Gyro : ±250°/s (131 LSB/°/s) -> Highest sensitivity for turning/yaw integration
// - DLPF : 21 Hz -> Eliminates motor & chassis vibration noise while keeping low latency (~8.5ms)
#define ACCEL_RANGE MPU6050_RANGE_2_G
#define GYRO_RANGE  MPU6050_RANGE_250_DEG
#define FILTER_BW   MPU6050_BAND_21_HZ

// Target raw values for hardware calibration
const int TARGET_ACCEL_X = 0;
const int TARGET_ACCEL_Y = 0;
const int TARGET_ACCEL_Z = 16384; // 1g at ±2g (16384 LSB/g)
const int TARGET_GYRO    = 0;

const int ACCEL_DEADZONE = 8;
const int GYRO_DEADZONE  = 1;

// Hardware offset values (stored in MPU6050 silicon registers)
int16_t accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
int16_t gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// Software offset values (for software subtraction method in esp_main)
float swGyroX_offset = 0.0f; // rad/s
float swGyroY_offset = 0.0f; // rad/s
float swGyroZ_offset = 0.0f; // rad/s

// ======================== CALIBRATION STATE MACHINE ==========================
enum CalibState {
  STATE_IDLE,
  STATE_STARTING,
  STATE_CALIBRATING,
  STATE_COMPLETE,
  STATE_FAILED
};

CalibState calibState = STATE_IDLE;
int calibIteration = 0;
const int MAX_CALIB_ITERATIONS = 50;
String calibLog = "Ready. Place robot on a flat, level surface and click 'Start Calibration'.";
unsigned long lastCalibStepMs = 0;
unsigned long calibStartTime = 0;

// ======================== I2C AUTO-RECOVERY & MPU HARDWARE ===================
void resetI2CBus() {
  Wire.end();
  delay(10);

  // Bit-bang 9 clock pulses on SCL to clear any stuck I2C slave
  pinMode(I2C_SDA_PIN, OUTPUT);
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, HIGH);
  digitalWrite(I2C_SCL_PIN, HIGH);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setTimeOut(25);
  Wire.setClock(400000);
  delay(15);
}

void configureSensor() {
  mpu.setAccelerometerRange(ACCEL_RANGE);
  mpu.setGyroRange(GYRO_RANGE);
  mpu.setFilterBandwidth(FILTER_BW);
}

bool tryInitMPU() {
  resetI2CBus();
  if (mpu.begin()) {
    mpuConnected = true;
    mpuFailCount = 0;
    configureSensor();
    Serial.println(F("✅ [MPU6050] Connected and configured successfully!"));
    calibLog = "MPU6050 connected. Place on a level surface and click 'Start Calibration'.";
    return true;
  }
  mpuConnected = false;
  return false;
}

void writeRegister(uint8_t reg, uint8_t value) {
  if (!mpuConnected) return;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

bool getRawValues(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz) {
  if (!mpuConnected) {
    *ax = *ay = *az = *gx = *gy = *gz = 0;
    return false;
  }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    mpuFailCount++;
    return false;
  }

  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);
  if (Wire.available() >= 14) {
    *ax = (Wire.read() << 8) | Wire.read();
    *ay = (Wire.read() << 8) | Wire.read();
    *az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip temperature bytes
    *gx = (Wire.read() << 8) | Wire.read();
    *gy = (Wire.read() << 8) | Wire.read();
    *gz = (Wire.read() << 8) | Wire.read();
    mpuFailCount = 0;
    return true;
  }

  mpuFailCount++;
  return false;
}

void writeOffsets() {
  if (!mpuConnected) return;

  // Read existing Temperature Compensation (TC) bit from low byte registers (Bit 0)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x07);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)1);
  uint8_t tc_xa = Wire.available() ? (Wire.read() & 0x01) : 0;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x09);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)1);
  uint8_t tc_ya = Wire.available() ? (Wire.read() & 0x01) : 0;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x0B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)1);
  uint8_t tc_za = Wire.available() ? (Wire.read() & 0x01) : 0;

  // Write accelerometer offsets, strictly preserving TC bits
  writeRegister(0x06, accelOffsetX >> 8);
  writeRegister(0x07, (accelOffsetX & 0xFE) | tc_xa);
  writeRegister(0x08, accelOffsetY >> 8);
  writeRegister(0x09, (accelOffsetY & 0xFE) | tc_ya);
  writeRegister(0x0A, accelOffsetZ >> 8);
  writeRegister(0x0B, (accelOffsetZ & 0xFE) | tc_za);

  // Write gyroscope hardware offsets
  writeRegister(0x13, gyroOffsetX >> 8);
  writeRegister(0x14, gyroOffsetX & 0xFF);
  writeRegister(0x15, gyroOffsetY >> 8);
  writeRegister(0x16, gyroOffsetY & 0xFF);
  writeRegister(0x17, gyroOffsetZ >> 8);
  writeRegister(0x18, gyroOffsetZ & 0xFF);
}

void resetHardwareOffsets() {
  accelOffsetX = accelOffsetY = accelOffsetZ = 0;
  gyroOffsetX = gyroOffsetY = gyroOffsetZ = 0;
  writeOffsets();
}

// ======================== ASYNCHRONOUS CALIBRATION ENGINE ====================
void stepCalibration() {
  if (!mpuConnected) {
    calibState = STATE_FAILED;
    calibLog = "❌ Calibration aborted: MPU6050 disconnected!";
    return;
  }

  if (calibState == STATE_STARTING) {
    calibLog = "Initializing calibration. Zeroing existing offsets...";
    resetHardwareOffsets();
    calibIteration = 0;
    calibStartTime = millis();
    calibState = STATE_CALIBRATING;
    return;
  }

  if (calibState == STATE_CALIBRATING) {
    calibIteration++;
    
    // Average 100 raw samples
    int32_t axSum = 0, aySum = 0, azSum = 0;
    int32_t gxSum = 0, gySum = 0, gzSum = 0;
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
      int16_t ax, ay, az, gx, gy, gz;
      if (!getRawValues(&ax, &ay, &az, &gx, &gy, &gz)) {
        if (mpuFailCount >= 3) {
          mpuConnected = false;
          calibState = STATE_FAILED;
          calibLog = "❌ Calibration error: lost I2C communication with MPU6050.";
          return;
        }
      }
      axSum += ax; aySum += ay; azSum += az;
      gxSum += gx; gySum += gy; gzSum += gz;
      delayMicroseconds(1200);
    }

    int16_t meanAx = axSum / samples;
    int16_t meanAy = aySum / samples;
    int16_t meanAz = azSum / samples;
    int16_t meanGx = gxSum / samples;
    int16_t meanGy = gySum / samples;
    int16_t meanGz = gzSum / samples;

    char stepBuf[128];
    snprintf(stepBuf, sizeof(stepBuf), "Iter %d/%d: Accel=[%d, %d, %d] Gyro=[%d, %d, %d]",
             calibIteration, MAX_CALIB_ITERATIONS, meanAx, meanAy, meanAz, meanGx, meanGy, meanGz);
    calibLog = String(stepBuf);
    Serial.println(stepBuf);

    // Check if error is within deadzone tolerances
    bool accelDone = (abs(meanAx - TARGET_ACCEL_X) <= ACCEL_DEADZONE) &&
                     (abs(meanAy - TARGET_ACCEL_Y) <= ACCEL_DEADZONE) &&
                     (abs(meanAz - TARGET_ACCEL_Z) <= ACCEL_DEADZONE);

    bool gyroDone  = (abs(meanGx - TARGET_GYRO) <= GYRO_DEADZONE) &&
                     (abs(meanGy - TARGET_GYRO) <= GYRO_DEADZONE) &&
                     (abs(meanGz - TARGET_GYRO) <= GYRO_DEADZONE);

    if (accelDone && gyroDone) {
      calibState = STATE_COMPLETE;
      
      // Calculate final software bias offsets across 300 samples
      float gSumX = 0, gSumY = 0, gSumZ = 0;
      for (int i = 0; i < 300; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gSumX += g.gyro.x;
        gSumY += g.gyro.y;
        gSumZ += g.gyro.z;
        delay(2);
      }
      swGyroX_offset = gSumX / 300.0f;
      swGyroY_offset = gSumY / 300.0f;
      swGyroZ_offset = gSumZ / 300.0f;

      calibLog = "✅ Calibration Converged Successfully!";
      Serial.println(F("\n============================================="));
      Serial.println(F("✅ CALIBRATION COMPLETE!"));
      Serial.printf("Hardware Offsets: Accel=[%d, %d, %d] Gyro=[%d, %d, %d]\n",
                    accelOffsetX, accelOffsetY, accelOffsetZ, gyroOffsetX, gyroOffsetY, gyroOffsetZ);
      Serial.printf("Software Gyro Z Offset: %.5f rad/s (%.3f deg/s)\n",
                    swGyroZ_offset, swGyroZ_offset * (180.0f / PI));
      Serial.println(F("=============================================\n"));
      return;
    }

    // Step hardware offsets toward target
    if (abs(meanAx - TARGET_ACCEL_X) > ACCEL_DEADZONE) accelOffsetX -= (meanAx - TARGET_ACCEL_X) / 8;
    if (abs(meanAy - TARGET_ACCEL_Y) > ACCEL_DEADZONE) accelOffsetY -= (meanAy - TARGET_ACCEL_Y) / 8;
    if (abs(meanAz - TARGET_ACCEL_Z) > ACCEL_DEADZONE) accelOffsetZ -= (meanAz - TARGET_ACCEL_Z) / 8;

    if (abs(meanGx - TARGET_GYRO) > GYRO_DEADZONE) gyroOffsetX -= meanGx / 4;
    if (abs(meanGy - TARGET_GYRO) > GYRO_DEADZONE) gyroOffsetY -= meanGy / 4;
    if (abs(meanGz - TARGET_GYRO) > GYRO_DEADZONE) gyroOffsetZ -= meanGz / 4;

    writeOffsets();

    if (calibIteration >= MAX_CALIB_ITERATIONS) {
      calibState = STATE_COMPLETE;
      
      float gSumZ = 0;
      for (int i = 0; i < 200; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gSumZ += g.gyro.z;
        delay(2);
      }
      swGyroZ_offset = gSumZ / 200.0f;

      calibLog = "⚠️ Completed with maximum iterations. Offsets saved.";
    }
  }
}

// ======================== HTML / JS WEB DASHBOARD ===========================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>MPU6050 Calibration & Live Monitor</title>
  <style>
    :root {
      --bg: #0d1117;
      --card-bg: #161b22;
      --card-border: #30363d;
      --text: #c9d1d9;
      --heading: #f0f6fc;
      --accent: #58a6ff;
      --green: #2ea043;
      --green-hover: #3fb950;
      --orange: #d29922;
      --red: #f85149;
      --purple: #bc8cff;
      --code-bg: #090d13;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }
    body { background-color: var(--bg); color: var(--text); padding: 16px; display: flex; justify-content: center; }
    .container { width: 100%; max-width: 900px; display: flex; flex-direction: column; gap: 16px; }
    
    header {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 12px;
      padding: 20px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-wrap: wrap;
      gap: 12px;
    }
    h1 { font-size: 20px; color: var(--heading); display: flex; align-items: center; gap: 10px; }
    .badge {
      font-size: 12px;
      padding: 4px 10px;
      border-radius: 20px;
      font-weight: 600;
      background: #1f2937;
      color: #9ca3af;
      border: 1px solid #374151;
    }
    .badge.online { background: #064e3b; color: #6ee7b7; border-color: #059669; }
    .badge.offline { background: #4c1d1d; color: #fca5a5; border-color: #dc2626; animation: pulse 1.5s infinite; }
    .badge.calib { background: #78350f; color: #fde68a; border-color: #d97706; animation: pulse 1.5s infinite; }
    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.6; } }

    .alert-banner {
      display: none;
      background: #381014;
      border: 1px solid #f85149;
      border-radius: 8px;
      padding: 14px 18px;
      color: #ffb4ab;
      font-size: 13px;
      line-height: 1.5;
    }
    .alert-banner.show { display: block; }

    .card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 12px;
      padding: 20px;
    }
    .card-title {
      font-size: 15px;
      font-weight: 600;
      color: var(--heading);
      margin-bottom: 14px;
      display: flex;
      align-items: center;
      justify-content: space-between;
    }

    .grid-3 { display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 14px; }
    .metric-box {
      background: #0d1117;
      border: 1px solid #21262d;
      border-radius: 8px;
      padding: 14px;
    }
    .metric-title { font-size: 12px; color: #8b949e; text-transform: uppercase; letter-spacing: 0.5px; margin-bottom: 6px; }
    .metric-val { font-size: 22px; font-weight: 700; color: var(--heading); font-family: monospace; }
    .metric-sub { font-size: 12px; color: #8b949e; margin-top: 4px; font-family: monospace; }

    .btn-calib {
      width: 100%;
      background: var(--green);
      color: white;
      border: none;
      padding: 16px;
      font-size: 16px;
      font-weight: 700;
      border-radius: 8px;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 10px;
      transition: all 0.2s;
    }
    .btn-calib:hover { background: var(--green-hover); }
    .btn-calib:disabled { background: #374151; color: #9ca3af; cursor: not-allowed; opacity: 0.6; }

    .progress-bar-bg {
      width: 100%;
      height: 10px;
      background: #21262d;
      border-radius: 6px;
      overflow: hidden;
      margin-top: 12px;
    }
    .progress-bar-fill {
      height: 100%;
      width: 0%;
      background: linear-gradient(90deg, var(--accent), var(--green));
      transition: width 0.3s ease;
    }

    .log-box {
      background: var(--code-bg);
      border: 1px solid #21262d;
      border-radius: 8px;
      padding: 12px;
      font-family: monospace;
      font-size: 13px;
      color: #58a6ff;
      min-height: 48px;
      display: flex;
      align-items: center;
    }

    .code-container {
      position: relative;
      background: var(--code-bg);
      border: 1px solid #30363d;
      border-radius: 8px;
      padding: 16px;
      margin-top: 10px;
    }
    pre {
      font-family: Consolas, Monaco, "Courier New", monospace;
      font-size: 13px;
      color: #7ee787;
      overflow-x: auto;
      white-space: pre-wrap;
    }
    .copy-btn {
      position: absolute;
      top: 10px;
      right: 10px;
      background: #21262d;
      color: #c9d1d9;
      border: 1px solid #30363d;
      padding: 6px 12px;
      border-radius: 6px;
      font-size: 12px;
      cursor: pointer;
    }
    .copy-btn:hover { background: #30363d; }

    .config-tag {
      display: inline-block;
      padding: 4px 8px;
      background: #1f2937;
      border-radius: 6px;
      font-size: 12px;
      color: var(--accent);
      margin-right: 6px;
      font-family: monospace;
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <div>
        <h1>⚖️ MPU6050 Offset Calibration</h1>
        <div style="font-size: 12px; color: #8b949e; margin-top: 4px;">ESP32 I2C (SDA: 21, SCL: 22) &bull; SoftAP Mode</div>
      </div>
      <div id="statusBadge" class="badge online">SENSOR ONLINE</div>
    </header>

    <!-- Disconnected Alert Banner -->
    <div id="disconnectAlert" class="alert-banner">
      <strong>⚠️ MPU6050 NOT DETECTED ON I2C BUS (GPIO 21 & 22)</strong><br>
      The ESP32 is actively scanning and auto-recovering the I2C bus in the background. Please verify your sensor wiring:
      <ul style="margin-left: 20px; margin-top: 6px;">
        <li><strong>VCC</strong> &rarr; 3.3V or 5V (ensure stable power)</li>
        <li><strong>GND</strong> &rarr; Common GND</li>
        <li><strong>SDA</strong> &rarr; ESP32 GPIO 21</li>
        <li><strong>SCL</strong> &rarr; ESP32 GPIO 22</li>
      </ul>
      <em>This page will automatically detect the sensor and start streaming data the moment it is reconnected!</em>
    </div>

    <!-- Calibration Control Box -->
    <div class="card">
      <div class="card-title">
        <span>🚀 Hardware & Software Calibration</span>
        <span id="iterText" style="font-size: 13px; color: #8b949e;">Idle</span>
      </div>
      <p style="font-size: 13px; color: #8b949e; margin-bottom: 14px;">
        Place the robot or sensor on a <strong>completely flat, stationary surface</strong> before clicking Start. Do not touch or move the sensor during the process.
      </p>
      <button id="calibBtn" class="btn-calib" onclick="startCalibration()">
        <span>🎯 Start MPU6050 Calibration</span>
      </button>
      <div class="progress-bar-bg">
        <div id="progressFill" class="progress-bar-fill"></div>
      </div>
      <div style="margin-top: 12px;" class="log-box" id="logBox">
        Ready. Place robot level and click 'Start Calibration'.
      </div>
    </div>

    <!-- Live Sensor Readings (Streaming) -->
    <div class="card">
      <div class="card-title">
        <span>📊 Live Sensor Telemetry (Streaming)</span>
        <span id="rateText" style="font-size: 12px; color: #58a6ff;">Rate: ~5 Hz</span>
      </div>
      <div class="grid-3">
        <!-- Accelerometer Box -->
        <div class="metric-box">
          <div class="metric-title">📐 Accelerometer (g / m/s²)</div>
          <div class="metric-val" id="accelVal">X: 0.00 | Y: 0.00</div>
          <div class="metric-sub" id="accelZVal">Z: 1.00 g (Earth Gravity)</div>
          <div class="metric-sub" id="rawAccel">Raw: [0, 0, 16384]</div>
        </div>
        <!-- Gyroscope Box -->
        <div class="metric-box">
          <div class="metric-title">🔄 Gyroscope (°/s - Rotation)</div>
          <div class="metric-val" id="gyroVal">Z: 0.00 °/s (Yaw)</div>
          <div class="metric-sub" id="gyroXYVal">X: 0.00 °/s | Y: 0.00 °/s</div>
          <div class="metric-sub" id="rawGyro">Raw: [0, 0, 0]</div>
        </div>
        <!-- Sensor Status & Temp -->
        <div class="metric-box">
          <div class="metric-title">🌡️ Sensor Core & Temperature</div>
          <div class="metric-val" id="tempVal">-- °C</div>
          <div class="metric-sub" id="hwOffsetsSummary">HW Offsets: [0, 0, 0, 0, 0, 0]</div>
          <div class="metric-sub" id="dlpfStatus" style="color: #2ea043;">DLPF Filter: 21 Hz active</div>
        </div>
      </div>
    </div>

    <!-- Recommended Code Snippet Output -->
    <div class="card">
      <div class="card-title">
        <span>📋 Calibrated Offsets for <code style="color:var(--accent);">esp_main.ino</code></span>
      </div>
      <p style="font-size: 13px; color: #8b949e;">
        When calibration finishes, copy and paste this snippet directly into your robot's main sketch:
      </p>
      <div class="code-container">
        <button class="copy-btn" onclick="copyCode()">📋 Copy</button>
        <pre id="codeSnippet">// Run calibration above to generate calibrated offset values...</pre>
      </div>
    </div>

    <!-- Active Hardware Config Info -->
    <div class="card">
      <div class="card-title">
        <span>⚙️ Active Sensor Configuration</span>
      </div>
      <div style="display: flex; flex-wrap: wrap; gap: 8px;">
        <span class="config-tag">Accel: ±2g (16384 LSB/g)</span>
        <span class="config-tag">Gyro: ±250°/s (131 LSB/°/s)</span>
        <span class="config-tag">DLPF: 21 Hz (Anti-Vibration)</span>
        <span class="config-tag">Auto-Recovery: Active (SCL Unstick)</span>
      </div>
    </div>
  </div>

  <script>
    let isCalibrating = false;

    function startCalibration() {
      if (isCalibrating) return;
      if (!confirm("Is the robot placed on a flat, level surface and completely still?")) return;
      
      fetch('/api/calibrate')
        .then(res => res.json())
        .then(data => {
          document.getElementById('calibBtn').disabled = true;
          document.getElementById('statusBadge').className = 'badge calib';
          document.getElementById('statusBadge').innerText = 'CALIBRATING...';
        })
        .catch(err => console.error(err));
    }

    function copyCode() {
      const code = document.getElementById('codeSnippet').innerText;
      navigator.clipboard.writeText(code).then(() => {
        alert("✅ Copied to clipboard!");
      });
    }

    function updateTelemetry() {
      fetch('/api/data')
        .then(r => r.json())
        .then(d => {
          const alertEl = document.getElementById('disconnectAlert');
          const badge = document.getElementById('statusBadge');
          const calibBtn = document.getElementById('calibBtn');

          if (!d.mpuConnected) {
            alertEl.className = 'alert-banner show';
            badge.className = 'badge offline';
            badge.innerText = 'SENSOR DISCONNECTED';
            calibBtn.disabled = true;
            document.getElementById('iterText').innerText = 'Reconnecting...';
            document.getElementById('logBox').innerText = '⚠️ MPU6050 not found on I2C bus. Auto-reconnecting on GPIO 21 & 22...';
            document.getElementById('accelVal').innerText = 'X: -- | Y: --';
            document.getElementById('accelZVal').innerText = 'Z: -- g';
            document.getElementById('rawAccel').innerText = 'Raw: [--, --, --]';
            document.getElementById('gyroVal').innerText = 'Z: -- °/s';
            document.getElementById('gyroXYVal').innerText = 'X: -- | Y: --';
            document.getElementById('rawGyro').innerText = 'Raw: [--, --, --]';
            document.getElementById('tempVal').innerText = '-- °C';
            document.getElementById('dlpfStatus').innerText = 'I2C Bus: Retrying...';
            document.getElementById('dlpfStatus').style.color = '#f85149';
            return;
          }

          // Sensor is connected
          alertEl.className = 'alert-banner';
          document.getElementById('dlpfStatus').innerText = 'DLPF Filter: 21 Hz active';
          document.getElementById('dlpfStatus').style.color = '#2ea043';

          // Live readings
          document.getElementById('accelVal').innerText = `X: ${d.ax.toFixed(2)} | Y: ${d.ay.toFixed(2)}`;
          document.getElementById('accelZVal').innerText = `Z: ${d.az.toFixed(2)} g (${(d.az*9.81).toFixed(2)} m/s²)`;
          document.getElementById('rawAccel').innerText = `Raw: [${d.rawAx}, ${d.rawAy}, ${d.rawAz}]`;

          document.getElementById('gyroVal').innerText = `Z: ${d.gz.toFixed(2)} °/s (Yaw Rate)`;
          document.getElementById('gyroXYVal').innerText = `X: ${d.gx.toFixed(2)} | Y: ${d.gy.toFixed(2)} °/s`;
          document.getElementById('rawGyro').innerText = `Raw: [${d.rawGx}, ${d.rawGy}, ${d.rawGz}]`;

          document.getElementById('tempVal').innerText = `${d.temp.toFixed(1)} °C`;
          document.getElementById('hwOffsetsSummary').innerText = `HW: A[${d.oxA}, ${d.oyA}, ${d.ozA}] G[${d.oxG}, ${d.oyG}, ${d.ozG}]`;

          // Calibration state updates
          document.getElementById('logBox').innerText = d.log;
          
          if (d.state === 'CALIBRATING') {
            isCalibrating = true;
            calibBtn.disabled = true;
            badge.className = 'badge calib';
            badge.innerText = 'CALIBRATING...';
            document.getElementById('iterText').innerText = `Step ${d.iter}/50`;
            const pct = Math.min(100, Math.round((d.iter / 50) * 100));
            document.getElementById('progressFill').style.width = pct + '%';
          } else if (d.state === 'COMPLETE') {
            isCalibrating = false;
            calibBtn.disabled = false;
            badge.className = 'badge online';
            badge.innerText = 'CALIBRATION COMPLETE';
            document.getElementById('iterText').innerText = 'Converged';
            document.getElementById('progressFill').style.width = '100%';

            // Update code snippet
            const snippet = 
`// ─── Paste into esp_main.ino setup() or calibration section ───────────
// Software Gyro Z Zero-Rate Bias Offset:
const float CALIBRATED_GYRO_Z_OFFSET = ${d.swGz.toFixed(6)}f; // ${ (d.swGz * (180.0/Math.PI)).toFixed(3) } deg/s

// Hardware Silicon Register Offsets (Optional hardware zeroing):
// accelOffsetX = ${d.oxA}; accelOffsetY = ${d.oyA}; accelOffsetZ = ${d.ozA};
// gyroOffsetX  = ${d.oxG}; gyroOffsetY  = ${d.oyG}; gyroOffsetZ  = ${d.ozG};`;
            
            document.getElementById('codeSnippet').innerText = snippet;
          } else {
            isCalibrating = false;
            calibBtn.disabled = false;
            badge.className = 'badge online';
            badge.innerText = 'SENSOR ONLINE';
          }
        })
        .catch(err => console.error(err));
    }

    setInterval(updateTelemetry, 220);
  </script>
</body>
</html>
)rawliteral";

// ======================== API ROUTE HANDLERS =================================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleCalibrate() {
  if (!mpuConnected) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"MPU6050 not connected\"}");
    return;
  }
  calibState = STATE_STARTING;
  server.send(200, "application/json", "{\"status\":\"started\"}");
}

void handleData() {
  if (!mpuConnected) {
    char json[256];
    snprintf(json, sizeof(json),
      "{\"mpuConnected\":false,\"state\":\"DISCONNECTED\",\"log\":\"%s\"}",
      calibLog.c_str()
    );
    server.send(200, "application/json", json);
    return;
  }

  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  bool ok = getRawValues(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);
  if (!ok && mpuFailCount >= 3) {
    mpuConnected = false;
    server.send(200, "application/json", "{\"mpuConnected\":false,\"state\":\"DISCONNECTED\",\"log\":\"Lost connection to MPU6050!\"}");
    return;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  String stateStr = "IDLE";
  if (calibState == STATE_STARTING || calibState == STATE_CALIBRATING) stateStr = "CALIBRATING";
  else if (calibState == STATE_COMPLETE) stateStr = "COMPLETE";
  else if (calibState == STATE_FAILED) stateStr = "FAILED";

  float azG = rawAz / 16384.0f;
  float axG = rawAx / 16384.0f;
  float ayG = rawAy / 16384.0f;

  float gxDeg = (g.gyro.x) * (180.0f / PI);
  float gyDeg = (g.gyro.y) * (180.0f / PI);
  float gzDeg = (g.gyro.z) * (180.0f / PI);

  char json[512];
  snprintf(json, sizeof(json),
    "{\"mpuConnected\":true,\"state\":\"%s\",\"iter\":%d,\"log\":\"%s\","
    "\"rawAx\":%d,\"rawAy\":%d,\"rawAz\":%d,"
    "\"rawGx\":%d,\"rawGy\":%d,\"rawGz\":%d,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
    "\"temp\":%.1f,"
    "\"oxA\":%d,\"oyA\":%d,\"ozA\":%d,"
    "\"oxG\":%d,\"oyG\":%d,\"ozG\":%d,"
    "\"swGz\":%.6f}",
    stateStr.c_str(), calibIteration, calibLog.c_str(),
    rawAx, rawAy, rawAz,
    rawGx, rawGy, rawGz,
    axG, ayG, azG,
    gxDeg, gyDeg, gzDeg,
    temp.temperature,
    accelOffsetX, accelOffsetY, accelOffsetZ,
    gyroOffsetX, gyroOffsetY, gyroOffsetZ,
    swGyroZ_offset
  );

  server.send(200, "application/json", json);
}

// ======================== SETUP & LOOP =======================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║   Auto Waste Bin — MPU6050 Web Calibration Tool    ║");
  Serial.println("╚════════════════════════════════════════════════════╝\n");

  // 1. Attempt Initial I2C & MPU6050 Initialization (Non-blocking)
  if (tryInitMPU()) {
    Serial.println(F("✅ MPU6050 found on initial boot!"));
  } else {
    Serial.println(F("⚠️  MPU6050 not detected at startup — starting SoftAP and will auto-reconnect..."));
  }

  // 2. Start SoftAP (Always starts immediately, never blocked by missing sensor!)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress IP = WiFi.softAPIP();

  Serial.println(F("\n📡 Wi-Fi Access Point Started:"));
  Serial.printf("  SSID     : %s\n", AP_SSID);
  Serial.printf("  Password : %s\n", AP_PASS);
  Serial.printf("  Web URL  : http://%s\n\n", IP.toString().c_str());

  // 3. Configure Web Server Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/calibrate", HTTP_GET, handleCalibrate);
  server.begin();
  Serial.println(F("✅ Web Dashboard running. Open http://192.168.4.1 in your browser."));
}

void loop() {
  server.handleClient();

  unsigned long now = millis();

  // Background Auto-Reconnect Watchdog: if MPU is disconnected, retry every 1 second
  if (!mpuConnected) {
    if (now - lastReconnectAttempt >= 1000) {
      lastReconnectAttempt = now;
      tryInitMPU();
    }
  }

  // Run non-blocking calibration state machine when connected
  if (calibState == STATE_STARTING || calibState == STATE_CALIBRATING) {
    if (now - lastCalibStepMs >= 80) {
      lastCalibStepMs = now;
      stepCalibration();
    }
  }

  delay(2);
}