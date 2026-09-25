/*
 * =========================================================================================
 * DUAL-MODE WEB INTERFACE & SYSTEM CONTROLLER (ESP32)
 * Project: Auto Waste Bin (Final Year Project)
 * =========================================================================================
 * 
 * Features:
 * 1. Hosted Wi-Fi Access Point (SoftAP): SSID "Auto_Waste_Bin_AP" (192.168.4.1)
 * 2. Embedded Web Dashboard with Dual Operating Interfaces:
 *    - MANUAL CONTROL: D-Pad navigation, live speed & threshold tuning, independent
 *      servo controls for Ultrasonic Servo (GPIO 32) and Cam Servo (GPIO 33).
 *    - AUTONOMOUS MODE: Obstacle avoidance state machine (Forward -> Stop -> 500ms Reverse 
 *      -> Left/Right Scan -> Path comparison -> Decision -> Turn & Resume) with a real-time 
 *      Decision & Reasoning log window.
 * 3. Flipped Motor Logic: Corrected motor directions so forward moves forward and backward moves reverse.
 * 4. Safety Systems: Automatic obstacle stop when driving forward in manual mode.
 * 5. MPU6050 Gyro integration for real-time Yaw angle tracking + Zeroing feature.
 * 
 * Pinout Summary (ESP32):
 * -----------------------------------------------------------------------------------------
 * Motor A (Left)     : ENA (GPIO 25), IN1 (GPIO 26), IN2 (GPIO 27)
 * Motor B (Right)    : ENB (GPIO 13), IN3 (GPIO 14), IN4 (GPIO 19)
 * Ultrasonic Servo   : Servo 1 (GPIO 32)
 * Cam Servo          : Servo 2 (GPIO 33)
 * Ultrasonic Sensor  : TRIG (GPIO 4), ECHO (GPIO 35 - Input only)
 * MPU6050 IMU (I2C)  : SDA (GPIO 21), SCL (GPIO 22)
 * =========================================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Wire.h>

// ======================== PIN DEFINITIONS ========================
// Motor A (Left)
#define PIN_ENA 25
#define PIN_IN1 26
#define PIN_IN2 27

// Motor B (Right)
#define PIN_ENB 13
#define PIN_IN3 14
#define PIN_IN4 19

// Servos
#define SERVO_ULTRASONIC_PIN 32
#define SERVO_CAM_PIN        33

// Ultrasonic Sensor
#define TRIG_PIN 4
#define ECHO_PIN 35

// MPU6050 I2C Pins
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// ======================== OBJECTS & CONSTANTS ====================
WebServer server(80);
Adafruit_MPU6050 mpu;
Servo ultrasonicServo;
Servo camServo;

const float SOUND_SPEED = 0.0343; // cm/microsecond

// Access Point Credentials
const char* AP_SSID = "Auto_Waste_Bin_AP";
const char* AP_PASS = "12345678";

// ======================== GLOBAL VARIABLES =======================
// System Modes
enum SystemMode { MODE_MANUAL, MODE_AUTONOMOUS };
SystemMode currentMode = MODE_MANUAL;

// Autonomous State Machine
enum AutoState {
  AUTO_IDLE,
  AUTO_DRIVE,
  AUTO_OBSTACLE_DETECTED,
  AUTO_BACKUP,
  AUTO_SCAN_LEFT,
  AUTO_SCAN_RIGHT,
  AUTO_DECIDE,
  AUTO_TURNING
};
AutoState currentAutoState = AUTO_IDLE;

// Adjustable System Parameters (Modifiable via Web Interface)
float obstacleThresholdCm = 20.0;
int defaultDriveSpeed     = 110;
int defaultTurnSpeed      = 80;

// Motor State
String currentMotorState = "STOPPED";
int currentSpeedA = 0;
int currentSpeedB = 0;

// Servo Positions
int ultrasonicServoPos = 90;
int camServoPos        = 90;

// Sensor Data
float latestDistanceCm = -1.0;
sensors_event_t accelEvent, gyroEvent, tempEvent;
bool mpuConnected = false;

// MPU Gyro Integration
float gyroZ_offset = 0.0;
float currentYaw = 0.0;
unsigned long lastSampleTimeMicros = 0;

// Autonomous Scan Data
float leftScanDist  = -1.0;
float rightScanDist = -1.0;
unsigned long autoTimer = 0;
int autoTurnTargetDuration = 600; // ms

// Decision Logging Buffer (for Autonomous Web Interface Window)
#define MAX_LOGS 20
String decisionLogs[MAX_LOGS];
int logCount = 0;

// Keep-Alive / Deadman Safety Timer for Manual Drive
unsigned long lastManualCmdMillis = 0;
const unsigned long MANUAL_TIMEOUT_MS = 2000;

// Timing intervals
unsigned long lastSensorReadMillis = 0;

// ======================== LOGGING FUNCTION =======================
void addDecisionLog(String message) {
  unsigned long seconds = millis() / 1000;
  unsigned long mins = seconds / 60;
  seconds %= 60;
  
  char timeStr[12];
  snprintf(timeStr, sizeof(timeStr), "[%02lu:%02lu] ", mins, seconds);
  String formattedLog = String(timeStr) + message;

  Serial.println(formattedLog);

  if (logCount < MAX_LOGS) {
    decisionLogs[logCount++] = formattedLog;
  } else {
    // Shift logs up
    for (int i = 0; i < MAX_LOGS - 1; i++) {
      decisionLogs[i] = decisionLogs[i + 1];
    }
    decisionLogs[MAX_LOGS - 1] = formattedLog;
  }
}

// ======================== MOTOR FUNCTIONS ========================
// NOTE: Motor directions flipped per instructions so Forward moves robot Forward.
void setMotorSpeeds(int speedA, int speedB) {
  currentSpeedA = constrain(speedA, 0, 255);
  currentSpeedB = constrain(speedB, 0, 255);
  analogWrite(PIN_ENA, currentSpeedA);
  analogWrite(PIN_ENB, currentSpeedB);
}

void stopMotors() {
  currentMotorState = "STOPPED";
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(0, 0);
}

void moveForward(int speed = defaultDriveSpeed) {
  currentMotorState = "FORWARD";
  // Motor pins set for Forward movement
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  setMotorSpeeds(speed, speed);
}

void moveBackward(int speed = defaultDriveSpeed) {
  currentMotorState = "BACKWARD";
  // Motor pins set for Backward movement
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(speed, speed);
}

void turnLeft(int speed = defaultTurnSpeed) {
  currentMotorState = "TURNING LEFT";
  // Left motor Reverse, Right motor Forward -> Turns Left
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  setMotorSpeeds(speed, speed);
}

void turnRight(int speed = defaultTurnSpeed) {
  currentMotorState = "TURNING RIGHT";
  // Left motor Forward, Right motor Reverse -> Turns Right
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(speed, speed);
}

// ======================== SERVO FUNCTIONS ========================
void setUltrasonicServo(int angle) {
  ultrasonicServoPos = constrain(angle, 0, 180);
  ultrasonicServo.write(ultrasonicServoPos);
}

void setCamServo(int angle) {
  camServoPos = constrain(angle, 0, 180);
  camServo.write(camServoPos);
}

// ======================== SENSOR READINGS ========================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m max)
  if (duration == 0) {
    return -1.0;
  }
  return (duration * SOUND_SPEED) / 2.0;
}

void updateMPU() {
  if (!mpuConnected) return;

  unsigned long now = micros();
  float dt = (now - lastSampleTimeMicros) / 1000000.0;
  lastSampleTimeMicros = now;

  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  float gyroZ_deg_s = (gyroEvent.gyro.z - gyroZ_offset) * (180.0 / PI);

  // Deadband noise filter
  if (abs(gyroZ_deg_s) > 0.6) {
    currentYaw += gyroZ_deg_s * dt;
  }
}

void calibrateMPU() {
  if (!mpuConnected) return;
  Serial.println("[!] Calibrating MPU6050 Gyro... Keep robot still!");
  float sumZ = 0.0;
  const int samples = 300;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sumZ += g.gyro.z;
    delay(3);
  }

  gyroZ_offset = sumZ / samples;
  Serial.printf("✅ MPU6050 Calibrated! Offset: %.4f deg/s\n", gyroZ_offset * (180.0 / PI));
}

// ======================== AUTONOMOUS STATE MACHINE ================
void processAutonomousMode() {
  if (currentMode != MODE_AUTONOMOUS) return;

  unsigned long currentMillis = millis();

  switch (currentAutoState) {
    case AUTO_IDLE:
      addDecisionLog("🤖 Autonomous Drive Started. Facing Forward (90°)...");
      setUltrasonicServo(90);
      currentAutoState = AUTO_DRIVE;
      break;

    case AUTO_DRIVE:
      // Drive forward continuously while checking ultrasonic
      setUltrasonicServo(90);
      moveForward(defaultDriveSpeed);

      if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
        stopMotors();
        addDecisionLog("🚨 OBSTACLE DETECTED at " + String(latestDistanceCm, 1) + " cm! Stopping motors.");
        autoTimer = currentMillis;
        currentAutoState = AUTO_OBSTACLE_DETECTED;
      }
      break;

    case AUTO_OBSTACLE_DETECTED:
      // Small pause before reversing
      if (currentMillis - autoTimer >= 200) {
        addDecisionLog("⬅️ Reversing robot for 500 ms to create clearance...");
        moveBackward(defaultDriveSpeed);
        autoTimer = currentMillis;
        currentAutoState = AUTO_BACKUP;
      }
      break;

    case AUTO_BACKUP:
      // Reversing for 500ms
      if (currentMillis - autoTimer >= 500) {
        stopMotors();
        addDecisionLog("🛑 Reverse complete. Starting environment scan (Left & Right)...");
        setUltrasonicServo(160); // Turn servo LEFT
        autoTimer = currentMillis;
        currentAutoState = AUTO_SCAN_LEFT;
      }
      break;

    case AUTO_SCAN_LEFT:
      // Wait for servo to settle (350ms) then read distance
      if (currentMillis - autoTimer >= 350) {
        leftScanDist = readUltrasonic();
        String distStr = (leftScanDist > 0) ? (String(leftScanDist, 1) + " cm") : "Clear/Infinity";
        addDecisionLog("🔍 Scanned LEFT (160°): " + distStr);

        setUltrasonicServo(20); // Turn servo RIGHT
        autoTimer = currentMillis;
        currentAutoState = AUTO_SCAN_RIGHT;
      }
      break;

    case AUTO_SCAN_RIGHT:
      // Wait for servo to settle (350ms) then read distance
      if (currentMillis - autoTimer >= 350) {
        rightScanDist = readUltrasonic();
        String distStr = (rightScanDist > 0) ? (String(rightScanDist, 1) + " cm") : "Clear/Infinity";
        addDecisionLog("🔍 Scanned RIGHT (20°): " + distStr);

        setUltrasonicServo(90); // Reset servo to CENTER
        currentAutoState = AUTO_DECIDE;
      }
      break;

    case AUTO_DECIDE:
      // Decision logic based on left vs right clearance
      {
        float lDist = (leftScanDist <= 0) ? 999.0 : leftScanDist;
        float rDist = (rightScanDist <= 0) ? 999.0 : rightScanDist;

        if (lDist > rDist && lDist > obstacleThresholdCm) {
          addDecisionLog("💡 DECISION: Turning LEFT towards clearer path (" + 
                         String(lDist, 1) + " cm vs " + String(rDist, 1) + " cm).");
          turnLeft(defaultTurnSpeed);
          autoTurnTargetDuration = 650;
        } else if (rDist >= lDist && rDist > obstacleThresholdCm) {
          addDecisionLog("💡 DECISION: Turning RIGHT towards clearer path (" + 
                         String(rDist, 1) + " cm vs " + String(lDist, 1) + " cm).");
          turnRight(defaultTurnSpeed);
          autoTurnTargetDuration = 650;
        } else {
          addDecisionLog("⚠️ DECISION: Both directions blocked! Executing 180° U-Turn...");
          turnRight(defaultTurnSpeed);
          autoTurnTargetDuration = 1200;
        }

        autoTimer = currentMillis;
        currentAutoState = AUTO_TURNING;
      }
      break;

    case AUTO_TURNING:
      if (currentMillis - autoTimer >= (unsigned long)autoTurnTargetDuration) {
        stopMotors();
        addDecisionLog("✅ Turn maneuver complete. Resuming forward autonomous drive.");
        currentAutoState = AUTO_DRIVE;
      }
      break;
  }
}

// ======================== HTML DASHBOARD SOURCE ===================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Auto Waste Bin - Control Center</title>
  <style>
    :root {
      --bg-color: #0d1117;
      --card-bg: rgba(22, 27, 34, 0.8);
      --card-border: rgba(255, 255, 255, 0.1);
      --accent: #2f81f7;
      --accent-hover: #58a6ff;
      --green: #238636;
      --red: #da3633;
      --orange: #d29922;
      --text: #c9d1d9;
      --text-muted: #8b949e;
      --terminal-bg: #090d13;
    }
    
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }
    body { background-color: var(--bg-color); color: var(--text); padding: 15px; min-height: 100vh; display: flex; flex-direction: column; align-items: center; }
    
    .container { max-width: 900px; width: 100%; display: flex; flex-direction: column; gap: 20px; }
    
    header { background: var(--card-bg); border: 1px solid var(--card-border); padding: 20px; border-radius: 14px; text-align: center; backdrop-filter: blur(10px); }
    header h1 { font-size: 1.6rem; color: #fff; margin-bottom: 5px; }
    header p { color: var(--text-muted); font-size: 0.9rem; }
    
    /* Navigation Tabs */
    .tabs { display: flex; gap: 10px; background: rgba(0,0,0,0.3); padding: 6px; border-radius: 10px; border: 1px solid var(--card-border); }
    .tab-btn { flex: 1; padding: 12px; border: none; border-radius: 8px; background: transparent; color: var(--text-muted); font-size: 1rem; font-weight: 600; cursor: pointer; transition: all 0.2s ease; }
    .tab-btn.active { background: var(--accent); color: #fff; box-shadow: 0 4px 12px rgba(47, 129, 247, 0.3); }
    
    /* Layout Cards */
    .card { background: var(--card-bg); border: 1px solid var(--card-border); border-radius: 14px; padding: 20px; backdrop-filter: blur(10px); }
    .card h2 { font-size: 1.1rem; color: #fff; margin-bottom: 15px; display: flex; align-items: center; gap: 8px; border-bottom: 1px solid var(--card-border); padding-bottom: 8px; }
    
    /* D-Pad Controls */
    .dpad-container { display: flex; flex-direction: column; align-items: center; gap: 10px; margin: 15px 0; }
    .dpad-row { display: flex; gap: 10px; }
    .btn { background: #21262d; border: 1px solid #30363d; color: #fff; padding: 14px 22px; border-radius: 10px; font-size: 1rem; font-weight: bold; cursor: pointer; transition: all 0.15s ease; user-select: none; touch-action: manipulation; }
    .btn:active, .btn.pressed { background: var(--accent); transform: scale(0.96); }
    .btn-stop { background: var(--red); border-color: #f85149; }
    .btn-stop:active { background: #b62324; }
    
    /* Sliders & Controls */
    .control-group { display: flex; flex-direction: column; gap: 12px; }
    .slider-card { display: flex; flex-direction: column; gap: 6px; background: rgba(0,0,0,0.2); padding: 12px; border-radius: 8px; border: 1px solid rgba(255,255,255,0.05); }
    .slider-header { display: flex; justify-content: space-between; font-size: 0.9rem; font-weight: 500; }
    input[type=range] { width: 100%; accent-color: var(--accent); cursor: pointer; }
    
    .preset-btns { display: flex; gap: 8px; margin-top: 4px; }
    .btn-sm { padding: 6px 12px; font-size: 0.8rem; border-radius: 6px; background: #30363d; border: none; color: #fff; cursor: pointer; }
    .btn-sm:hover { background: var(--accent); }
    
    /* Telemetry Grid */
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; }
    .stat-box { background: rgba(0,0,0,0.3); border: 1px solid var(--card-border); padding: 12px; border-radius: 10px; text-align: center; }
    .stat-val { font-size: 1.3rem; font-weight: bold; color: #fff; margin-top: 4px; }
    .stat-label { font-size: 0.8rem; color: var(--text-muted); text-transform: uppercase; }
    
    /* Decision Log Terminal */
    .terminal { background: var(--terminal-bg); border: 1px solid #30363d; border-radius: 10px; padding: 15px; height: 260px; overflow-y: auto; font-family: "Courier New", Courier, monospace; font-size: 0.85rem; color: #7ee787; display: flex; flex-direction: column; gap: 4px; }
    .log-entry { border-bottom: 1px solid rgba(255,255,255,0.03); padding-bottom: 2px; }
    
    /* Status Badge */
    .badge { display: inline-block; padding: 4px 10px; border-radius: 12px; font-size: 0.8rem; font-weight: bold; }
    .badge-auto { background: rgba(210, 153, 34, 0.2); color: var(--orange); border: 1px solid var(--orange); }
    .badge-manual { background: rgba(47, 129, 247, 0.2); color: var(--accent); border: 1px solid var(--accent); }
    
    /* Responsive Grid Layout */
    .two-col { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
    @media (max-width: 700px) { .two-col { grid-template-columns: 1fr; } }
  </style>
</head>
<body>

<div class="container">
  <header>
    <h1>🗑️ Auto Waste Bin Control Center</h1>
    <p>ESP32 SoftAP Interface | Real-Time Telemetry & Dual Controls</p>
  </header>

  <!-- Mode Switcher Tabs -->
  <div class="tabs">
    <button class="tab-btn active" id="tab-manual-btn" onclick="switchMode('manual')">🎮 Manual Control</button>
    <button class="tab-btn" id="tab-auto-btn" onclick="switchMode('auto')">🤖 Autonomous Drive Mode</button>
  </div>

  <!-- Telemetry Bar -->
  <div class="grid">
    <div class="stat-box">
      <div class="stat-label">System Mode</div>
      <div class="stat-val" id="tele-mode"><span class="badge badge-manual">MANUAL</span></div>
    </div>
    <div class="stat-box">
      <div class="stat-label">Obstacle Distance</div>
      <div class="stat-val" id="tele-dist">-- cm</div>
    </div>
    <div class="stat-box">
      <div class="stat-label">MPU Yaw Angle</div>
      <div class="stat-val" id="tele-yaw">0.0°</div>
    </div>
    <div class="stat-box">
      <div class="stat-label">Motor State</div>
      <div class="stat-val" id="tele-motor">STOPPED</div>
    </div>
  </div>

  <!-- TAB 1: MANUAL CONTROL INTERFACE -->
  <div id="manual-view">
    <div class="two-col">
      <!-- Drive D-Pad Card -->
      <div class="card">
        <h2>🎮 Driving Controls</h2>
        <div class="dpad-container">
          <button class="btn" onmousedown="sendDrive('forward')" onmouseup="sendDrive('stop')" ontouchstart="sendDrive('forward')" ontouchend="sendDrive('stop')">▲ FORWARD</button>
          <div class="dpad-row">
            <button class="btn" onmousedown="sendDrive('left')" onmouseup="sendDrive('stop')" ontouchstart="sendDrive('left')" ontouchend="sendDrive('stop')">◀ LEFT</button>
            <button class="btn btn-stop" onclick="sendDrive('stop')">🛑 STOP</button>
            <button class="btn" onmousedown="sendDrive('right')" onmouseup="sendDrive('stop')" ontouchstart="sendDrive('right')" ontouchend="sendDrive('stop')">RIGHT ▶</button>
          </div>
          <button class="btn" onmousedown="sendDrive('backward')" onmouseup="sendDrive('stop')" ontouchstart="sendDrive('backward')" ontouchend="sendDrive('stop')">▼ BACKWARD</button>
        </div>
        <p style="text-align:center; font-size:0.8rem; color:var(--text-muted);">Keyboard Shortcut: W/A/S/D to move, Space to stop</p>
      </div>

      <!-- Independent Servo Control Card -->
      <div class="card">
        <h2>🤖 Independent Servos</h2>
        <div class="control-group">
          <!-- Ultrasonic Servo -->
          <div class="slider-card">
            <div class="slider-header">
              <span>📡 Ultrasonic Servo (GPIO 32)</span>
              <span id="us-servo-val">90°</span>
            </div>
            <input type="range" id="us-servo-slider" min="0" max="180" value="90" oninput="updateUSServo(this.value)">
            <div class="preset-btns">
              <button class="btn-sm" onclick="setUSServo(0)">0° (Right)</button>
              <button class="btn-sm" onclick="setUSServo(90)">90° (Center)</button>
              <button class="btn-sm" onclick="setUSServo(180)">180° (Left)</button>
            </div>
          </div>

          <!-- Cam Servo -->
          <div class="slider-card">
            <div class="slider-header">
              <span>📷 Cam Servo (GPIO 33)</span>
              <span id="cam-servo-val">90°</span>
            </div>
            <input type="range" id="cam-servo-slider" min="0" max="180" value="90" oninput="updateCamServo(this.value)">
            <div class="preset-btns">
              <button class="btn-sm" onclick="setCamServo(0)">0°</button>
              <button class="btn-sm" onclick="setCamServo(90)">90° (Center)</button>
              <button class="btn-sm" onclick="setCamServo(180)">180°</button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- Parameter Tuning Card -->
    <div class="card" style="margin-top: 15px;">
      <h2>⚙️ System Settings & Speed Tuning</h2>
      <div class="grid" style="grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));">
        <div class="slider-card">
          <div class="slider-header">
            <span>Drive Speed PWM</span>
            <span id="val-drive-speed">110</span>
          </div>
          <input type="range" id="slider-drive-speed" min="50" max="255" value="110" onchange="saveSettings()">
        </div>

        <div class="slider-card">
          <div class="slider-header">
            <span>Turn Speed PWM</span>
            <span id="val-turn-speed">80</span>
          </div>
          <input type="range" id="slider-turn-speed" min="50" max="255" value="80" onchange="saveSettings()">
        </div>

        <div class="slider-card">
          <div class="slider-header">
            <span>Obstacle Threshold</span>
            <span id="val-threshold">20 cm</span>
          </div>
          <input type="range" id="slider-threshold" min="5" max="50" value="20" onchange="saveSettings()">
        </div>
      </div>
      <div style="margin-top: 12px; display: flex; justify-content: flex-end; gap: 10px;">
        <button class="btn-sm" style="background:#21262d;" onclick="resetYaw()">🔄 Reset MPU Yaw to 0°</button>
      </div>
    </div>
  </div>

  <!-- TAB 2: AUTONOMOUS DRIVE INTERFACE -->
  <div id="auto-view" style="display: none;">
    <div class="card">
      <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:12px;">
        <h2>🧠 Autonomous Decision & Reasoning Window</h2>
        <button class="btn-sm" style="background:var(--red);" onclick="switchMode('manual')">🛑 Pause Autonomous Drive</button>
      </div>
      
      <!-- Terminal Console Output -->
      <div class="terminal" id="decision-terminal">
        <div class="log-entry">Waiting for autonomous telemetry logs...</div>
      </div>
    </div>
  </div>
</div>

<script>
  let currentMode = 'manual';
  
  function switchMode(mode) {
    currentMode = mode;
    fetch(`/api/cmd?mode=${mode}`)
      .then(res => res.json())
      .then(data => updateUI(data));
      
    if(mode === 'manual') {
      document.getElementById('manual-view').style.display = 'block';
      document.getElementById('auto-view').style.display = 'none';
      document.getElementById('tab-manual-btn').classList.add('active');
      document.getElementById('tab-auto-btn').classList.remove('active');
    } else {
      document.getElementById('manual-view').style.display = 'none';
      document.getElementById('auto-view').style.display = 'block';
      document.getElementById('tab-manual-btn').classList.remove('active');
      document.getElementById('tab-auto-btn').classList.add('active');
    }
  }

  function sendDrive(dir) {
    if(currentMode !== 'manual') return;
    fetch(`/api/cmd?dir=${dir}`);
  }

  function updateUSServo(val) {
    document.getElementById('us-servo-val').innerText = val + '°';
    fetch(`/api/cmd?us_servo=${val}`);
  }
  function setUSServo(val) {
    document.getElementById('us-servo-slider').value = val;
    updateUSServo(val);
  }

  function updateCamServo(val) {
    document.getElementById('cam-servo-val').innerText = val + '°';
    fetch(`/api/cmd?cam_servo=${val}`);
  }
  function setCamServo(val) {
    document.getElementById('cam-servo-slider').value = val;
    updateCamServo(val);
  }

  function saveSettings() {
    let driveSpeed = document.getElementById('slider-drive-speed').value;
    let turnSpeed  = document.getElementById('slider-turn-speed').value;
    let threshold  = document.getElementById('slider-threshold').value;
    
    document.getElementById('val-drive-speed').innerText = driveSpeed;
    document.getElementById('val-turn-speed').innerText  = turnSpeed;
    document.getElementById('val-threshold').innerText   = threshold + ' cm';
    
    fetch(`/api/settings?drive_speed=${driveSpeed}&turn_speed=${turnSpeed}&threshold=${threshold}`);
  }

  function resetYaw() {
    fetch('/api/cmd?reset_yaw=1');
  }

  // Keyboard Navigation
  document.addEventListener('keydown', (e) => {
    if(currentMode !== 'manual') return;
    if(e.repeat) return;
    if(e.key === 'w' || e.key === 'W') sendDrive('forward');
    if(e.key === 's' || e.key === 'S') sendDrive('backward');
    if(e.key === 'a' || e.key === 'A') sendDrive('left');
    if(e.key === 'd' || e.key === 'D') sendDrive('right');
    if(e.key === ' ') sendDrive('stop');
  });

  document.addEventListener('keyup', (e) => {
    if(currentMode !== 'manual') return;
    if(['w','W','s','S','a','A','d','D'].includes(e.key)) sendDrive('stop');
  });

  // Telemetry Polling
  function pollTelemetry() {
    fetch('/api/telemetry')
      .then(res => res.json())
      .then(data => updateUI(data))
      .catch(err => console.log('Polling error:', err));
  }

  function updateUI(data) {
    if(!data) return;
    
    // Telemetry updates
    document.getElementById('tele-mode').innerHTML = data.mode === 'AUTONOMOUS' 
      ? '<span class="badge badge-auto">AUTONOMOUS</span>' 
      : '<span class="badge badge-manual">MANUAL</span>';
      
    document.getElementById('tele-dist').innerText = (data.ultrasonic >= 0) ? data.ultrasonic.toFixed(1) + ' cm' : 'Out of Range';
    document.getElementById('tele-yaw').innerText  = (data.mpu_yaw >= 0 ? '+' : '') + data.mpu_yaw.toFixed(1) + '°';
    document.getElementById('tele-motor').innerText = data.motor_state;

    // Terminal log updates for Autonomous view
    if(data.logs && data.logs.length > 0) {
      let term = document.getElementById('decision-terminal');
      term.innerHTML = data.logs.map(log => `<div class="log-entry">${log}</div>`).join('');
      term.scrollTop = term.scrollHeight;
    }
  }

  setInterval(pollTelemetry, 300);
</script>
</body>
</html>
)rawliteral";

// ======================== WEB ROUTE HANDLERS ======================
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleTelemetry() {
  String json = "{";
  json += "\"mode\":\"" + String((currentMode == MODE_AUTONOMOUS) ? "AUTONOMOUS" : "MANUAL") + "\",";
  json += "\"ultrasonic\":" + String(latestDistanceCm) + ",";
  json += "\"mpu_yaw\":" + String(currentYaw, 2) + ",";
  json += "\"motor_state\":\"" + currentMotorState + "\",";
  json += "\"drive_speed\":" + String(defaultDriveSpeed) + ",";
  json += "\"turn_speed\":" + String(defaultTurnSpeed) + ",";
  json += "\"threshold\":" + String(obstacleThresholdCm, 1) + ",";
  json += "\"us_servo\":" + String(ultrasonicServoPos) + ",";
  json += "\"cam_servo\":" + String(camServoPos) + ",";
  
  json += "\"logs\":[";
  for (int i = 0; i < logCount; i++) {
    json += "\"" + decisionLogs[i] + "\"";
    if (i < logCount - 1) json += ",";
  }
  json += "]";
  json += "}";

  server.send(200, "application/json", json);
}

void handleCommand() {
  lastManualCmdMillis = millis();

  if (server.hasArg("mode")) {
    String modeArg = server.arg("mode");
    if (modeArg == "auto") {
      currentMode = MODE_AUTONOMOUS;
      currentAutoState = AUTO_IDLE;
      addDecisionLog("🔄 Mode Switched to AUTONOMOUS MODE");
    } else {
      currentMode = MODE_MANUAL;
      stopMotors();
      addDecisionLog("🔄 Mode Switched to MANUAL CONTROL");
    }
  }

  if (currentMode == MODE_MANUAL && server.hasArg("dir")) {
    String dir = server.arg("dir");
    if (dir == "forward") {
      // Safety obstacle check
      if (latestDistanceCm > 0 && latestDistanceCm < obstacleThresholdCm) {
        stopMotors();
        addDecisionLog("🚨 Cannot move forward: Obstacle closer than " + String(obstacleThresholdCm, 1) + " cm!");
      } else {
        moveForward(defaultDriveSpeed);
      }
    } else if (dir == "backward") {
      moveBackward(defaultDriveSpeed);
    } else if (dir == "left") {
      turnLeft(defaultTurnSpeed);
    } else if (dir == "right") {
      turnRight(defaultTurnSpeed);
    } else if (dir == "stop") {
      stopMotors();
    }
  }

  if (server.hasArg("us_servo")) {
    int angle = server.arg("us_servo").toInt();
    setUltrasonicServo(angle);
  }

  if (server.hasArg("cam_servo")) {
    int angle = server.arg("cam_servo").toInt();
    setCamServo(angle);
  }

  if (server.hasArg("reset_yaw")) {
    currentYaw = 0.0;
    addDecisionLog("🔄 MPU6050 Yaw Angle zeroed.");
  }

  handleTelemetry();
}

void handleSettings() {
  if (server.hasArg("drive_speed")) {
    defaultDriveSpeed = server.arg("drive_speed").toInt();
  }
  if (server.hasArg("turn_speed")) {
    defaultTurnSpeed = server.arg("turn_speed").toInt();
  }
  if (server.hasArg("threshold")) {
    obstacleThresholdCm = server.arg("threshold").toFloat();
  }

  addDecisionLog("⚙️ Settings Updated: Drive=" + String(defaultDriveSpeed) + 
                 " | Turn=" + String(defaultTurnSpeed) + 
                 " | Threshold=" + String(obstacleThresholdCm, 1) + "cm");

  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// ======================== SETUP ==================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n============================================================");
  Serial.println("   AUTO WASTE BIN - DUAL MODE CONTROL & SOFT AP WEB SERVER  ");
  Serial.println("============================================================");

  // 1. Allocate ESP32 PWM Timers & Attach Servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  ultrasonicServo.setPeriodHertz(50);
  camServo.setPeriodHertz(50);

  if (ultrasonicServo.attach(SERVO_ULTRASONIC_PIN, 500, 2400)) {
    Serial.printf("✅ Ultrasonic Servo Attached on GPIO %d\n", SERVO_ULTRASONIC_PIN);
  }
  if (camServo.attach(SERVO_CAM_PIN, 500, 2400)) {
    Serial.printf("✅ Cam Servo Attached on GPIO %d\n", SERVO_CAM_PIN);
  }

  setUltrasonicServo(90);
  setCamServo(90);

  // 2. Configure Motor Driver Pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  stopMotors();

  // 3. Configure Ultrasonic Sensor Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // 4. Configure MPU6050 I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!mpu.begin()) {
    Serial.println("⚠️ MPU6050 not detected on I2C pins (SDA=21, SCL=22). Continuing without IMU...");
    mpuConnected = false;
  } else {
    Serial.println("✅ MPU6050 Connected successfully!");
    mpuConnected = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    calibrateMPU();
  }

  // 5. Start Wi-Fi SoftAP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress apIP = WiFi.softAPIP();

  Serial.println("\n🌐 Wi-Fi SoftAP Started!");
  Serial.printf("   SSID     : %s\n", AP_SSID);
  Serial.printf("   Password : %s\n", AP_PASS);
  Serial.printf("   IP Address: http://%s\n\n", apIP.toString().c_str());

  // 6. Setup Web Server Endpoints
  server.on("/", handleRoot);
  server.on("/api/telemetry", handleTelemetry);
  server.on("/api/cmd", handleCommand);
  server.on("/api/settings", handleSettings);
  server.begin();
  Serial.println("✅ Web Server Running on Port 80");

  lastSampleTimeMicros = micros();
  lastSensorReadMillis = millis();
  lastManualCmdMillis  = millis();

  addDecisionLog("🚀 System Initialization Complete. Ready for connections.");
}

// ======================== MAIN LOOP ==============================
void loop() {
  // 1. High frequency MPU Gyro integration
  updateMPU();

  // 2. Handle HTTP client requests
  server.handleClient();

  // 3. Read Ultrasonic Sensor every 100ms
  if (millis() - lastSensorReadMillis >= 100) {
    lastSensorReadMillis = millis();
    latestDistanceCm = readUltrasonic();

    // Safety feature in MANUAL mode: auto-stop if moving forward into obstacle
    if (currentMode == MODE_MANUAL && currentMotorState == "FORWARD" &&
        latestDistanceCm > 0 && latestDistanceCm < obstacleThresholdCm) {
      stopMotors();
      addDecisionLog("🚨 [MANUAL SAFETY STOP] Obstacle detected closer than " + String(obstacleThresholdCm, 1) + " cm!");
    }
  }

  // 4. Process Autonomous state machine
  processAutonomousMode();
}
