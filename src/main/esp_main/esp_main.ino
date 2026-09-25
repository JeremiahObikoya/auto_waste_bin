/*
 * =============================================================================
 * AUTO WASTE BIN V2 — MAIN ESP32 NAVIGATION NODE
 * =============================================================================
 * Board   : ESP32 Dev Module (30-pin or 38-pin)
 * Role    : Motor control, IMU heading, ultrasonic safety, SoftAP web
 *           dashboard, and Wi-Fi UDP interface with the ESP32-CAM Vision Node.
 *
 * Pinout Summary:
 *   Motor A (Left)   : ENA (GPIO 25), IN1 (GPIO 26), IN2 (GPIO 27)
 *   Motor B (Right)  : ENB (GPIO 13), IN3 (GPIO 14), IN4 (GPIO 19)
 *   Ultrasonic Servo : GPIO 32
 *   Camera Pan Servo : GPIO 33  ← on this ESP32, NOT on the ESP32-CAM
 *   Ultrasonic Sensor: TRIG (GPIO 4), ECHO (GPIO 35 — input only)
 *   MPU6050 (I2C)   : SDA (GPIO 21), SCL (GPIO 22)
 *
 * Operating Modes (set via web dashboard):
 *   0 MODE_MANUAL          – SoftAP D-pad control with obstacle safety stop
 *   1 MODE_AUTONOMOUS      – 3-Classroom Patrol state machine
 *   2 MODE_CAM_GUIDED      – Wi-Fi UDP vision-guided with Gemini AI
 *
 * Required Libraries:
 *   Adafruit MPU6050, Adafruit Unified Sensor, ESP32Servo
 * =============================================================================
 */

#include "secrets.h" // ← Credentials & UDP ports
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

Preferences prefs;

// ======================== PIN DEFINITIONS ====================================
#define PIN_ENA 25
#define PIN_IN1 26
#define PIN_IN2 27
#define PIN_ENB 13
#define PIN_IN3 14
#define PIN_IN4 19
#define SERVO_ULTRASONIC 32
#define SERVO_ULTRASONIC_PIN 32 // Alias for convenience
#define SERVO_CAM_PIN 33        // Camera pan servo — on THIS ESP32
#define TRIG_PIN 4
#define ECHO_PIN 35 // Input-only GPIO
#define I2C_SDA 21
#define I2C_SCL 22

// Camera servo pan angles for scan cycle: Front/Center (90°), Left (45°), Right
// (135°)
#define CAM_ANGLE_CENTER 90 // Center / Front
#define CAM_ANGLE_LEFT 45   // Left pan
#define CAM_ANGLE_RIGHT 135 // Right pan

// ======================== OBJECTS & CONSTANTS ================================
WebServer server(80);
Adafruit_MPU6050 mpu;
Servo ultrasonicServo;
Servo camServo;
WiFiUDP udp;

const float SOUND_SPEED = 0.0343f; // cm / µs
#define MANUAL_TIMEOUT_MS 350
unsigned long lastManualCmdMs = 0;

// ======================== SYSTEM MODES =======================================
enum SystemMode : uint8_t {
  MODE_MANUAL = 0,
  MODE_AUTONOMOUS = 1, // 3-Classroom Patrol
  MODE_CAM_GUIDED = 2  // Vision-guided with Gemini
};
SystemMode currentMode = MODE_MANUAL;

enum ManualDriveState {
  MANUAL_IDLE,
  MANUAL_DRIVE_FORWARD,
  MANUAL_DRIVE_BACKWARD,
  MANUAL_DRIVE_CIRCLE_LEFT,
  MANUAL_DRIVE_CIRCLE_RIGHT,
  MANUAL_OBSTACLE_AVOID
};
ManualDriveState manualDriveState = MANUAL_IDLE;
ManualDriveState manualResumeState = MANUAL_IDLE;

// ======================== GLOBAL SENSOR / MOTOR STATE ========================
float latestDistanceCm = -1.0f;
char currentMotorState[16] = "STOPPED";
int currentSpeedA = 0;
int currentSpeedB = 0;
int ultrasonicPos = 90;

// ======================== SYSTEM PARAMETERS & NETWORK CONFIG =================
float obstacleThresholdCm = 50.0f; // Safety stop distance (cm)
int defaultDriveSpeed = 110;
int turnLeftSpeed =
    180; // Calibrated minimum static friction breakaway for LEFT (180 PWM)
int turnRightSpeed =
    210; // Calibrated minimum static friction breakaway for RIGHT (210 PWM)
int camSteerGain = 2;              // Proportional gain for curve-steering
float headingToleranceDeg = 12.0f; // Runtime tunable angle lock tolerance (deg)
char currentLaptopIp[32] = "10.218.193.152";
char currentWifiSsid[33] = WIFI_SSID;
char currentWifiPass[65] = WIFI_PASSWORD;

// ======================== DECISION LOG RING BUFFER ===========================
#define MAX_LOGS 25
#define MAX_LOG_LEN 96
char decisionLogs[MAX_LOGS][MAX_LOG_LEN];
int logHead = 0;
int totalLogsAdded = 0;

void addLog(const char *msg) {
  unsigned long s = millis() / 1000;
  char timeBuf[16];
  snprintf(timeBuf, sizeof(timeBuf), "[%02lu:%02lu] ", s / 60, s % 60);

  char fullEntry[MAX_LOG_LEN];
  snprintf(fullEntry, sizeof(fullEntry), "%s%s", timeBuf, msg);

  Serial.println(fullEntry);

  strncpy(decisionLogs[logHead], fullEntry, MAX_LOG_LEN - 1);
  decisionLogs[logHead][MAX_LOG_LEN - 1] = '\0';

  logHead = (logHead + 1) % MAX_LOGS;
  totalLogsAdded++;
}

void addLog(const String &msg) { addLog(msg.c_str()); }

// ======================== MPU6050 CALIBRATED OFFSETS & I2C AUTO-RECOVERY =====
// Calibrated from MPU6050 Web Calibration Tool:
const float CALIBRATED_GYRO_Z_OFFSET = 0.000141f; // 0.008 deg/s zero-rate bias

const int16_t HW_ACCEL_OFFSET_X = -3556;
const int16_t HW_ACCEL_OFFSET_Y = 272;
const int16_t HW_ACCEL_OFFSET_Z = 1336;
const int16_t HW_GYRO_OFFSET_X = 91;
const int16_t HW_GYRO_OFFSET_Y = -17;
const int16_t HW_GYRO_OFFSET_Z = 12;

bool mpuConnected = false;
int mpuErrorCount = 0;
float gyroZ_offset = CALIBRATED_GYRO_Z_OFFSET;
float currentYaw = 0.0f;
unsigned long lastSampleUs = 0;
sensors_event_t accelEv, gyroEv, tempEv;

void writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void applyMPUHardwareOffsets() {
  if (!mpuConnected)
    return;

  // Read existing Temperature Compensation (TC) bits from low byte registers
  // (Bit 0)
  Wire.beginTransmission(0x68);
  Wire.write(0x07);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (size_t)1);
  uint8_t tc_xa = Wire.available() ? (Wire.read() & 0x01) : 0;

  Wire.beginTransmission(0x68);
  Wire.write(0x09);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (size_t)1);
  uint8_t tc_ya = Wire.available() ? (Wire.read() & 0x01) : 0;

  Wire.beginTransmission(0x68);
  Wire.write(0x0B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (size_t)1);
  uint8_t tc_za = Wire.available() ? (Wire.read() & 0x01) : 0;

  // Write accelerometer offsets, preserving TC bits
  writeMPURegister(0x06, HW_ACCEL_OFFSET_X >> 8);
  writeMPURegister(0x07, (HW_ACCEL_OFFSET_X & 0xFE) | tc_xa);
  writeMPURegister(0x08, HW_ACCEL_OFFSET_Y >> 8);
  writeMPURegister(0x09, (HW_ACCEL_OFFSET_Y & 0xFE) | tc_ya);
  writeMPURegister(0x0A, HW_ACCEL_OFFSET_Z >> 8);
  writeMPURegister(0x0B, (HW_ACCEL_OFFSET_Z & 0xFE) | tc_za);

  // Write gyroscope hardware offsets
  writeMPURegister(0x13, HW_GYRO_OFFSET_X >> 8);
  writeMPURegister(0x14, HW_GYRO_OFFSET_X & 0xFF);
  writeMPURegister(0x15, HW_GYRO_OFFSET_Y >> 8);
  writeMPURegister(0x16, HW_GYRO_OFFSET_Y & 0xFF);
  writeMPURegister(0x17, HW_GYRO_OFFSET_Z >> 8);
  writeMPURegister(0x18, HW_GYRO_OFFSET_Z & 0xFF);

  addLog("✅ MPU6050 calibrated hardware & software offsets loaded.");
}

void calibrateMPU() {
  if (!mpuConnected)
    return;
  addLog("🔧 Fine-tuning MPU6050 offset — keep robot still...");
  float sum = 0;
  float minGz = 999.0f, maxGz = -999.0f;
  for (int i = 0; i < 300; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sum += g.gyro.z;
    if (g.gyro.z < minGz)
      minGz = g.gyro.z;
    if (g.gyro.z > maxGz)
      maxGz = g.gyro.z;
    delay(3);
  }
  // Check if robot was moving during calibration (gyro spread > 0.08 rad/s
  // ~ 4.5°/s)
  if ((maxGz - minGz) > 0.08f) {
    addLog("⚠️ Movement detected during calibration! Retrying in 250ms...");
    delay(250);
    sum = 0;
    for (int i = 0; i < 300; i++) {
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);
      sum += g.gyro.z;
      delay(3);
    }
  }
  gyroZ_offset = sum / 300.0f;
  char offBuf[64];
  snprintf(offBuf, sizeof(offBuf),
           "✅ MPU6050 calibrated. Offset: %.5f rad/s (%.3f °/s)", gyroZ_offset,
           gyroZ_offset * (180.0f / PI));
  addLog(offBuf);
}

void resetI2C() {
  addLog("⚠️ [MPU6050] 3 Consecutive I2C errors! Resetting I2C bus...");
  Wire.end();
  delay(20);

  // Bit-bang 9 clocks on SCL to unstick slave
  pinMode(I2C_SDA, OUTPUT);
  pinMode(I2C_SCL, OUTPUT);
  digitalWrite(I2C_SDA, HIGH);
  digitalWrite(I2C_SCL, HIGH);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(25);
  delay(30);

  if (mpu.begin()) {
    mpuConnected = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    applyMPUHardwareOffsets();
    gyroZ_offset = CALIBRATED_GYRO_Z_OFFSET;
    addLog("✅ [MPU6050] I2C bus and sensor successfully recovered!");
  } else {
    mpuConnected = false;
    addLog("❌ [MPU6050] Sensor re-init failed — running in fallback mode.");
  }
  mpuErrorCount = 0;
}

float currentGyroRateZ = 0.0f;
float currentAccelJerk = 0.0f;
float prevAccelX = 0.0f, prevAccelY = 0.0f;

void updateMPU() {
  unsigned long now = micros();
  float dt = (now - lastSampleUs) / 1000000.0f;
  lastSampleUs = now;

  // Guard against initial boot lag or loop delay spikes
  if (dt <= 0.0f || dt > 0.05f)
    dt = 0.01f;

  if (!mpuConnected) {
    // Attempt auto-recovery every 3 seconds if disconnected
    static unsigned long lastAutoRecoverMs = 0;
    if (millis() - lastAutoRecoverMs >= 3000) {
      lastAutoRecoverMs = millis();
      resetI2C();
    }
    return;
  }

  bool ok = mpu.getEvent(&accelEv, &gyroEv, &tempEv);
  if (!ok || isnan(gyroEv.gyro.z) || isnan(accelEv.acceleration.x)) {
    mpuErrorCount++;
    if (mpuErrorCount >= 3) {
      resetI2C();
    }
    return;
  }
  mpuErrorCount = 0;

  float gz = (gyroEv.gyro.z - gyroZ_offset) * (180.0f / PI);
  currentGyroRateZ = gz;
  if (fabsf(gz) > 0.4f)
    currentYaw += gz * dt;

  // Track linear chassis vibration & acceleration jerk
  float dAx = fabsf(accelEv.acceleration.x - prevAccelX);
  float dAy = fabsf(accelEv.acceleration.y - prevAccelY);
  currentAccelJerk = (currentAccelJerk * 0.7f) + ((dAx + dAy) * 0.3f);
  prevAccelX = accelEv.acceleration.x;
  prevAccelY = accelEv.acceleration.y;
}

// ======================== SERVO JITTER FIX (AUTO-DETACH) =====================
int ultrasonicAngle = 90;
int camAngleWritten = CAM_ANGLE_CENTER;
bool ultrasonicAttached = false;
bool camAttached = false;
unsigned long ultrasonicMoveMs = 0;
unsigned long camMoveMs = 0;

void writeUltrasonicServo(int angle, bool force = false) {
  angle = constrain(angle, 0, 180);
  if (angle != ultrasonicAngle || force) {
    if (!ultrasonicAttached) {
      ultrasonicServo.attach(SERVO_ULTRASONIC, 500, 2400);
      ultrasonicAttached = true;
    }
    ultrasonicAngle = angle;
    ultrasonicPos = angle;
    ultrasonicServo.write(angle);
    ultrasonicMoveMs = millis();
  }
}

void writeCamServo(int angle, bool force = false) {
  angle = constrain(angle, 0, 180);
  if (angle != camAngleWritten || force) {
    if (!camAttached) {
      camServo.attach(SERVO_CAM_PIN, 500, 2400);
      camAttached = true;
    }
    camAngleWritten = angle;
    camServo.write(angle);
    camMoveMs = millis();
  }
}

void serviceServos() {
  unsigned long now = millis();
  if (ultrasonicAttached && (now - ultrasonicMoveMs >= 350)) {
    ultrasonicServo.detach();
    ultrasonicAttached = false;
  }
  if (camAttached && (now - camMoveMs >= 350)) {
    camServo.detach();
    camAttached = false;
  }
}

// ======================== ADAPTIVE STALL & VOLTAGE COMPENSATION ==============
// Automatically detects chassis stalls due to battery discharge or surface
// friction.
// - Turning: monitors Gyro Z angular rate & interval-based Yaw delta.
// - Driving: monitors Accelerometer motion surge/vibration.
// If robot is stalled over each check window, increases PWM by +12 step-by-step
// until motion is observed.
unsigned long stallCheckIntervalMs =
    400; // Global tunable stall check interval (default 400ms)
int stallBoostLeft = 0;
int stallBoostRight = 0;
int stallBoostDrive = 0;
int stallBoostCircleL = 0;
int stallBoostCircleR = 0;
unsigned long motionStateStartMs = 0;
unsigned long lastStallBoostMs = 0;
char lastObservedMotorState[16] = "STOPPED";
float lastRecordedYaw = 0.0f;
float lastRecordedAccelX = 0.0f;
float lastRecordedAccelY = 0.0f;
bool isStalled = false;

// ======================== MOTOR FUNCTIONS ====================================
void setMotorSpeeds(int sA, int sB) {
  currentSpeedA = constrain(sA, 0, 255);
  currentSpeedB = constrain(sB, 0, 255);
  analogWrite(PIN_ENA, currentSpeedA);
  analogWrite(PIN_ENB, currentSpeedB);
}

// Global non-blocking brake pulse controller
bool motorBrakingActive = false;
unsigned long motorBrakeStartMs = 0;
const unsigned long BRAKE_PULSE_DURATION_MS = 80;

void stopMotors() {
  motorBrakingActive = false; // Cancel any active brake pulse
  strncpy(currentMotorState, "STOPPED", sizeof(currentMotorState));
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(0, 0);
  motionStateStartMs = 0;
  lastStallBoostMs = 0;
  isStalled = false;
  stallBoostLeft = 0;
  stallBoostRight = 0;
  stallBoostDrive = 0;
  stallBoostCircleL = 0;
  stallBoostCircleR = 0;
}

// Non-blocking active electrical brake: drives all IN pins HIGH simultaneously
// to short the motor back-EMF, applying maximum magnetic stopping torque.
// Asynchronously serviced and released to coast by serviceBrake() after
// BRAKE_PULSE_DURATION_MS.
void brakeMotors() {
  strncpy(currentMotorState, "BRAKING", sizeof(currentMotorState));
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, HIGH);
  setMotorSpeeds(200, 200); // Lock motor shafts electrically
  motorBrakeStartMs = millis();
  motorBrakingActive = true;
  motionStateStartMs = 0;
  lastStallBoostMs = 0;
  isStalled = false;
}

// Non-blocking brake pulse release service (called every loop() iteration)
void serviceBrake() {
  if (motorBrakingActive &&
      (millis() - motorBrakeStartMs >= BRAKE_PULSE_DURATION_MS)) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    setMotorSpeeds(0, 0);
    motorBrakingActive = false;
    strncpy(currentMotorState, "STOPPED", sizeof(currentMotorState));
    motionStateStartMs = 0;
    lastStallBoostMs = 0;
    isStalled = false;
  }
}

void moveForward(int spd = 0) {
  if (spd == 0)
    spd = defaultDriveSpeed;
  int effectiveSpd = constrain(spd + stallBoostDrive, 0, 255);
  strncpy(currentMotorState, "FORWARD", sizeof(currentMotorState));
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  setMotorSpeeds(effectiveSpd, effectiveSpd);
}

void moveBackward(int spd = 0) {
  if (spd == 0)
    spd = defaultDriveSpeed;
  int effectiveSpd = constrain(spd + stallBoostDrive, 0, 255);
  strncpy(currentMotorState, "BACKWARD", sizeof(currentMotorState));
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(effectiveSpd, effectiveSpd);
}

void turnLeft(int spd = 0) {
  if (spd == 0)
    spd = turnLeftSpeed;
  int effectiveSpd = constrain(spd + stallBoostLeft, 0, 255);
  strncpy(currentMotorState, "LEFT", sizeof(currentMotorState));
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  setMotorSpeeds(effectiveSpd, effectiveSpd);
}

void turnRight(int spd = 0) {
  if (spd == 0)
    spd = turnRightSpeed;
  int effectiveSpd = constrain(spd + stallBoostRight, 0, 255);
  strncpy(currentMotorState, "RIGHT", sizeof(currentMotorState));
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(effectiveSpd, effectiveSpd);
}

void circleLeft(int spd = 0) {
  if (spd == 0)
    spd = turnLeftSpeed;
  int outerSpd = constrain(spd + stallBoostLeft + stallBoostCircleL, 210, 255);
  // Inner wheels (Left): Light reverse (48-70 PWM) to break 4WD tire scrub
  // while outer wheels (210-255 PWM) drive forward in a wide circle
  int innerRevSpd = constrain(48 + (stallBoostCircleL / 4), 45, 70);
  strncpy(currentMotorState, "CIRCLE_L", sizeof(currentMotorState));

  // Left Motors (A): REVERSE at light speed (inner pivot)
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);

  // Right Motors (B): FORWARD at high speed (outer arc)
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);

  setMotorSpeeds(innerRevSpd, outerSpd);
}

void circleRight(int spd = 0) {
  if (spd == 0)
    spd = turnRightSpeed;
  int outerSpd = constrain(spd + stallBoostRight + stallBoostCircleR, 210, 255);
  // Inner wheels (Right): Light reverse (48-70 PWM) to break 4WD tire scrub
  // while outer wheels (210-255 PWM) drive forward in a wide circle
  int innerRevSpd = constrain(48 + (stallBoostCircleR / 4), 45, 70);
  strncpy(currentMotorState, "CIRCLE_R", sizeof(currentMotorState));

  // Left Motors (A): FORWARD at high speed (outer arc)
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);

  // Right Motors (B): REVERSE at light speed (inner pivot)
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);

  setMotorSpeeds(outerSpd, innerRevSpd);
}

void curveSteer(int8_t angleOffset) {
  strncpy(currentMotorState, "TRACKING", sizeof(currentMotorState));
  int base = defaultDriveSpeed + stallBoostDrive;
  int left = constrain(base + angleOffset * camSteerGain, 40, 255);
  int right = constrain(base - angleOffset * camSteerGain, 40, 255);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, left);
  analogWrite(PIN_ENB, right);
}

// Active Stall Compensation Service (called in loop())
// Moving-window stall detector: checks progress every stallCheckIntervalMs
void serviceStallCompensation() {
  // Safety check: only execute if MPU is healthy, valid, and connected
  if (!mpuConnected || mpuErrorCount > 0 || isnan(gyroEv.gyro.z) ||
      isnan(accelEv.acceleration.x))
    return;

  // Persistent breakaway kick state variables for manual/service stall recovery
  static bool unwedgePulseActive = false;
  static unsigned long unwedgePulseStartMs = 0;
  static unsigned long maxPwmStallTimer = 0;

  // If stopped or braking, keep tracking reset
  if (strcmp(currentMotorState, "STOPPED") == 0 ||
      strcmp(currentMotorState, "BRAKING") == 0) {
    if (strcmp(lastObservedMotorState, "STOPPED") != 0) {
      strncpy(lastObservedMotorState, currentMotorState,
              sizeof(lastObservedMotorState));
      motionStateStartMs = 0;
      lastStallBoostMs = 0;
      isStalled = false;
      unwedgePulseActive = false;
      maxPwmStallTimer = 0;
    }
    return;
  }

  unsigned long now = millis();

  // If motion command state changed (e.g. STOPPED -> LEFT or FORWARD -> RIGHT)
  if (strcmp(currentMotorState, lastObservedMotorState) != 0) {
    strncpy(lastObservedMotorState, currentMotorState,
            sizeof(lastObservedMotorState));
    motionStateStartMs = now;
    lastStallBoostMs = now;
    lastRecordedYaw = currentYaw;
    lastRecordedAccelX = accelEv.acceleration.x;
    lastRecordedAccelY = accelEv.acceleration.y;
    isStalled = false;
    unwedgePulseActive = false;
    maxPwmStallTimer = 0;
    return;
  }

  // ── BREAKAWAY KICK IN FLIGHT (Non-blocking 180ms)
  // ───────────────────────────
  if (unwedgePulseActive) {
    if (now - unwedgePulseStartMs < 180) {
      return; // Let breakaway pulse complete
    }
    // Breakaway kick finished! Reapply active motion mode with high PWM
    unwedgePulseActive = false;
    maxPwmStallTimer = now;
    lastRecordedYaw = currentYaw;
    if (strcmp(currentMotorState, "LEFT") == 0) {
      turnLeft(255);
    } else if (strcmp(currentMotorState, "RIGHT") == 0) {
      turnRight(255);
    } else if (strcmp(currentMotorState, "CIRCLE_L") == 0) {
      circleLeft(turnLeftSpeed);
    } else if (strcmp(currentMotorState, "CIRCLE_R") == 0) {
      circleRight(turnRightSpeed);
    }
    return;
  }

  // Initial grace period for motor magnetic field to energize & attempt
  // movement
  if (now - motionStateStartMs < stallCheckIntervalMs)
    return;

  // Check moving window progress every stallCheckIntervalMs
  if (now - lastStallBoostMs < stallCheckIntervalMs)
    return;

  // ── A. TURNING STALL DETECTION (LEFT / RIGHT) ─────────────────────────────
  if (strcmp(currentMotorState, "LEFT") == 0 ||
      strcmp(currentMotorState, "RIGHT") == 0) {
    float deltaYawInterval = fabsf(currentYaw - lastRecordedYaw);
    float rotRate = fabsf(currentGyroRateZ);

    // If robot is stalled: very low angular velocity AND minimal yaw delta
    // during this interval
    if (rotRate < 3.5f && deltaYawInterval < 0.8f) {
      isStalled = true;
      if (strcmp(currentMotorState, "LEFT") == 0) {
        int maxBoost = 255 - turnLeftSpeed;
        if (stallBoostLeft < maxBoost) {
          stallBoostLeft = min(stallBoostLeft + 15, maxBoost);
          int newSpd =
              constrain(turnLeftSpeed + stallBoostLeft, turnLeftSpeed, 255);
          setMotorSpeeds(newSpd, newSpd);
          maxPwmStallTimer = now;
          Serial.printf(
              "⚡ [STALL BOOST -> LEFT] Yaw: %6.1f° stuck! Rate: %4.1f°/s, "
              "Delta: %4.1f° in %lums | Boost: +%d -> Applied PWM: %d\n",
              currentYaw, rotRate, deltaYawInterval, stallCheckIntervalMs,
              stallBoostLeft, newSpd);
        } else {
          // Reached MAXIMUM PWM and still stalled -> Trigger Force-Move
          // Breakaway Kick!
          if (maxPwmStallTimer == 0)
            maxPwmStallTimer = now;
          if (now - maxPwmStallTimer >= 600) {
            Serial.printf("🚨 [FORCE MOVE -> LEFT] Reached Max PWM (%d) and "
                          "still stalled! Applying 180ms Breakaway Kick...\n",
                          currentSpeedA);
            addLog("⚡ [FORCE MOVE] Maximum PWM reached on LEFT turn — "
                   "executing breakaway kick!");
            unwedgePulseActive = true;
            unwedgePulseStartMs = now;
            // Reverse kick pulse to unstick caster
            digitalWrite(PIN_IN1, LOW);
            digitalWrite(PIN_IN2, HIGH);
            digitalWrite(PIN_IN3, HIGH);
            digitalWrite(PIN_IN4, LOW);
            setMotorSpeeds(255, 255);
            return;
          }
        }
      } else { // RIGHT
        int maxBoost = 255 - turnRightSpeed;
        if (stallBoostRight < maxBoost) {
          stallBoostRight = min(stallBoostRight + 15, maxBoost);
          int newSpd =
              constrain(turnRightSpeed + stallBoostRight, turnRightSpeed, 255);
          setMotorSpeeds(newSpd, newSpd);
          maxPwmStallTimer = now;
          Serial.printf(
              "⚡ [STALL BOOST -> RIGHT] Yaw: %6.1f° stuck! Rate: %4.1f°/s, "
              "Delta: %4.1f° in %lums | Boost: +%d -> Applied PWM: %d\n",
              currentYaw, rotRate, deltaYawInterval, stallCheckIntervalMs,
              stallBoostRight, newSpd);
        } else {
          // Reached MAXIMUM PWM and still stalled -> Trigger Force-Move
          // Breakaway Kick!
          if (maxPwmStallTimer == 0)
            maxPwmStallTimer = now;
          if (now - maxPwmStallTimer >= 600) {
            Serial.printf("🚨 [FORCE MOVE -> RIGHT] Reached Max PWM (%d) and "
                          "still stalled! Applying 180ms Breakaway Kick...\n",
                          currentSpeedA);
            addLog("⚡ [FORCE MOVE] Maximum PWM reached on RIGHT turn — "
                   "executing breakaway kick!");
            unwedgePulseActive = true;
            unwedgePulseStartMs = now;
            // Reverse kick pulse to unstick caster
            digitalWrite(PIN_IN1, HIGH);
            digitalWrite(PIN_IN2, LOW);
            digitalWrite(PIN_IN3, LOW);
            digitalWrite(PIN_IN4, HIGH);
            setMotorSpeeds(255, 255);
            return;
          }
        }
      }
    } else if (rotRate >= 5.0f || deltaYawInterval >= 1.2f) {
      if (isStalled) {
        isStalled = false;
        maxPwmStallTimer = 0;
        Serial.printf("🚀 [MOTION RESUMED -> %s] Yaw: %6.1f° | Rate: %5.1f°/s "
                      "| Delta: %4.1f° | Boost: +%d | Holding PWM: %d\n",
                      currentMotorState, currentYaw, rotRate, deltaYawInterval,
                      (strcmp(currentMotorState, "LEFT") == 0
                           ? stallBoostLeft
                           : stallBoostRight),
                      currentSpeedA);
      }
    }

    lastStallBoostMs = now;
    lastRecordedYaw = currentYaw;
  }

  // ── B. CIRCLING STALL DETECTION (CIRCLE_L / CIRCLE_R) ──────────────────────
  else if (strcmp(currentMotorState, "CIRCLE_L") == 0 ||
           strcmp(currentMotorState, "CIRCLE_R") == 0) {
    float deltaYawInterval = fabsf(currentYaw - lastRecordedYaw);
    float rotRate = fabsf(currentGyroRateZ);

    // MPU6050 Verification: Must observe genuine turning angular velocity or
    // yaw delta for circular orbit
    bool isMovingCircle = (rotRate >= 4.0f || deltaYawInterval >= 1.0f);

    // If robot is stalled while trying to circle:
    if (!isMovingCircle) {
      isStalled = true;
      if (strcmp(currentMotorState, "CIRCLE_L") == 0) {
        int maxBoost = 255 - turnLeftSpeed;
        if (stallBoostCircleL < maxBoost) {
          stallBoostCircleL = min(stallBoostCircleL + 15, maxBoost);
          circleLeft(turnLeftSpeed);
          maxPwmStallTimer = now;
          Serial.printf("⚡ [STALL BOOST -> CIRCLE_L] Stalled! Rate: %4.1f°/s, "
                        "Delta: %4.1f° | Boost: +%d -> Applied PWM: (%d, %d)\n",
                        rotRate, deltaYawInterval, stallBoostCircleL,
                        currentSpeedA, currentSpeedB);
        } else {
          // Reached MAXIMUM PWM and still stalled -> Trigger Force-Move
          // Breakaway Kick!
          if (maxPwmStallTimer == 0)
            maxPwmStallTimer = now;
          if (now - maxPwmStallTimer >= 600) {
            Serial.printf(
                "🚨 [FORCE MOVE -> CIRCLE_L] Max PWM reached (%d, %d) but "
                "still stalled! Applying 180ms Breakaway Kick...\n",
                currentSpeedA, currentSpeedB);
            addLog("⚡ [FORCE MOVE] Maximum PWM reached while circling LEFT — "
                   "executing breakaway kick!");
            unwedgePulseActive = true;
            unwedgePulseStartMs = now;
            // Forceful pivot Left kick at 255 PWM to overcome static floor
            // friction
            digitalWrite(PIN_IN1, HIGH);
            digitalWrite(PIN_IN2, LOW);
            digitalWrite(PIN_IN3, LOW);
            digitalWrite(PIN_IN4, HIGH);
            setMotorSpeeds(255, 255);
            return;
          }
        }
      } else { // CIRCLE_R
        int maxBoost = 255 - turnRightSpeed;
        if (stallBoostCircleR < maxBoost) {
          stallBoostCircleR = min(stallBoostCircleR + 15, maxBoost);
          circleRight(turnRightSpeed);
          maxPwmStallTimer = now;
          Serial.printf("⚡ [STALL BOOST -> CIRCLE_R] Stalled! Rate: %4.1f°/s, "
                        "Delta: %4.1f° | Boost: +%d -> Applied PWM: (%d, %d)\n",
                        rotRate, deltaYawInterval, stallBoostCircleR,
                        currentSpeedA, currentSpeedB);
        } else {
          // Reached MAXIMUM PWM and still stalled -> Trigger Force-Move
          // Breakaway Kick!
          if (maxPwmStallTimer == 0)
            maxPwmStallTimer = now;
          if (now - maxPwmStallTimer >= 600) {
            Serial.printf(
                "🚨 [FORCE MOVE -> CIRCLE_R] Max PWM reached (%d, %d) but "
                "still stalled! Applying 180ms Breakaway Kick...\n",
                currentSpeedA, currentSpeedB);
            addLog("⚡ [FORCE MOVE] Maximum PWM reached while circling RIGHT — "
                   "executing breakaway kick!");
            unwedgePulseActive = true;
            unwedgePulseStartMs = now;
            // Forceful pivot Right kick at 255 PWM to overcome static floor
            // friction
            digitalWrite(PIN_IN1, LOW);
            digitalWrite(PIN_IN2, HIGH);
            digitalWrite(PIN_IN3, HIGH);
            digitalWrite(PIN_IN4, LOW);
            setMotorSpeeds(255, 255);
            return;
          }
        }
      }
    } else {
      // Robot HAS STARTED MOVING! Stop increasing PWM and hold steady.
      if (isStalled) {
        isStalled = false;
        maxPwmStallTimer = 0;
        Serial.printf("🚀 [MOTION RESUMED -> %s] Yaw: %6.1f° | Rate: %5.1f°/s "
                      "| Delta: %4.1f° | Holding PWM: (%d, %d)\n",
                      currentMotorState, currentYaw, rotRate, deltaYawInterval,
                      currentSpeedA, currentSpeedB);
      }
    }

    lastStallBoostMs = now;
    lastRecordedYaw = currentYaw;
    lastRecordedAccelX = accelEv.acceleration.x;
    lastRecordedAccelY = accelEv.acceleration.y;
  }

  // ── C. LINEAR DRIVE STALL DETECTION (FORWARD / BACKWARD) ──────────────────
  else if (strcmp(currentMotorState, "FORWARD") == 0 ||
           strcmp(currentMotorState, "BACKWARD") == 0) {
    float deltaAx = fabsf(accelEv.acceleration.x - lastRecordedAccelX);
    float deltaAy = fabsf(accelEv.acceleration.y - lastRecordedAccelY);
    float motionJerk = deltaAx + deltaAy;

    // If rolling chassis vibration or linear surge is absent (< 0.15 m/s^2):
    if (motionJerk < 0.15f) {
      stallBoostDrive = min(stallBoostDrive + 12, 255 - defaultDriveSpeed);
      int newSpd = constrain(defaultDriveSpeed + stallBoostDrive,
                             defaultDriveSpeed, 255);
      setMotorSpeeds(newSpd, newSpd);
      Serial.printf("⚡ [STALL BOOST -> DRIVE] Forward/Back sag! Boost: +%d -> "
                    "Applied PWM: %d\n",
                    stallBoostDrive, newSpd);
    } else if (motionJerk >= 0.25f) {
      if (isStalled) {
        isStalled = false;
      }
    }

    lastStallBoostMs = now;
    lastRecordedAccelX = accelEv.acceleration.x;
    lastRecordedAccelY = accelEv.acceleration.y;
  }
}

// ======================== SENSOR FUNCTIONS ===================================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  return (dur == 0) ? -1.0f : (dur * SOUND_SPEED) / 2.0f;
}

// 5-Ping Max Filter: takes numPings readings and returns the MAX valid distance
// detected. Max is the safest filter against acoustic deflection / false
// obstacles. Returns -1.0f only if all pings timed out.
float readUltrasonicMaxFilter(int numPings = 5, int pingIntervalMs = 15) {
  float maxDist = -1.0f;
  for (int i = 0; i < numPings; i++) {
    float d = readUltrasonic();
    if (d > 0.0f) {
      if (maxDist < 0.0f || d > maxDist) {
        maxDist = d;
      }
    }
    if (i < numPings - 1) {
      delay(pingIntervalMs);
    }
  }
  return maxDist;
}

bool isRobotMovingLinear() {
  if (!mpuConnected || mpuErrorCount > 0)
    return true; // Fallback if MPU offline
  return (currentAccelJerk >= 0.08f);
}

// ======================== UDP WI-FI ROUTER COMMUNICATION =====================
struct __attribute__((packed)) CamPacket {
  uint8_t
      state; // 0=SCANNING, 1=TRACKING, 2=INTERACTION, 3=RELEASE,
             // 4=SCAN_STEP_DONE, 5=OBSTACLE_CHECK, 6=IS_TARGET, 7=IS_OBSTACLE
  int8_t angleOffset;
};

struct __attribute__((packed)) MainPacket {
  uint8_t pktType; // 0=Mode/Heartbeat, 1=Config Sync, 5=Trigger Obstacle Check
  uint8_t systemMode; // 0=MANUAL, 1=AUTONOMOUS, 2=CAM_GUIDED, 3=PAUSED,
                      // 5=OBSTACLE_CHECK
  char laptopIp[32];
  char wifiSsid[33];
  char wifiPass[65];
};

CamPacket lastCamPkt = {0, 0};
bool newCamPkt = false;
unsigned long lastCamRxMs =
    0; // Timestamp of last received packet from ESP32-CAM
bool camPaused = false;

void broadcastMode(uint8_t pktType = 0) {
  MainPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.pktType = pktType;
  if (camPaused && currentMode == MODE_CAM_GUIDED)
    pkt.systemMode = 3; // PAUSED
  else if (pktType == 5)
    pkt.systemMode = 5; // OBSTACLE_CHECK
  else
    pkt.systemMode = (uint8_t)currentMode;
  strncpy(pkt.laptopIp, currentLaptopIp, sizeof(pkt.laptopIp));
  strncpy(pkt.wifiSsid, currentWifiSsid, sizeof(pkt.wifiSsid));
  strncpy(pkt.wifiPass, currentWifiPass, sizeof(pkt.wifiPass));

  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket("255.255.255.255", UDP_CAM_RX_PORT);
    udp.write((const uint8_t *)&pkt, sizeof(pkt));
    udp.endPacket();
  }
  Serial.printf("[UDP TX] systemMode=%d, laptopIp=%s, pktType=%d\n",
                pkt.systemMode, pkt.laptopIp, pktType);
}

void checkUdpPackets() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    if (packetSize == sizeof(CamPacket)) {
      udp.read((uint8_t *)&lastCamPkt, sizeof(CamPacket));
      newCamPkt = true;
      lastCamRxMs = millis();
      Serial.printf("[UDP RX] CamPacket: state=%d, angleOffset=%d\n",
                    lastCamPkt.state, lastCamPkt.angleOffset);
    } else {
      // Discard unexpected broadcast packets (e.g. router mDNS, PC discovery)
      // to prevent LwIP buffer leaks
      while (udp.available()) {
        udp.read();
      }
      udp.flush();
    }
  }
}

// ======================== CLOSED-LOOP YAW HEADING CONTROL ====================
// Precision Heading Control Algorithm:
//   1. Acceptance & Settle Window:
//      - When |targetAngle - currentYaw| <= tolerance, triggers non-blocking
//      active electrical brake.
//      - Settle dwell timer requires error to remain <= tolerance continuously
//      for SETTLE_WINDOW_MS (180ms)
//        AND the 80ms electrical brake pulse to complete before returning true.
//   2. Deceleration Zone (absErr < 30°):
//      - Dynamically scales down turn speed toward base breakaway speed.
//      - Dynamic stall boost (+stallBoost) is automatically added if static
//      friction prevents reaching target.
//   3. Far Zone (absErr >= 30°):
//      - Starts at base breakaway speed and smoothly ramps up (+25 PWM /
//      1000ms) up to 255 PWM.
//   4. Automatic Breakaway Kick / Unwedge Routine:
//      - If robot is stuck at maximum power (>245 PWM / max boost) for >=
//      1200ms with zero rotation (<0.8°),
//        executes a brief 180ms swing-pivot or reverse kick to overcome static
//        floor friction / caster bind.
bool rotateToHeading(float targetAngle, float tolerance = -1.0f) {
  // Use configured tolerance if none specified
  if (tolerance <= 0.0f)
    tolerance = headingToleranceDeg;

  // Enforce a safe floor (at least 1.0°) so a low NVS value never causes
  // oscillation
  if (tolerance < 1.0f)
    tolerance = 1.0f;

  // ── Hard Safety Interlock: Abort turn & halt motors if MPU is disconnected
  // ──
  if (!mpuConnected) {
    stopMotors();
    static unsigned long lastRotateWarnMs = 0;
    if (millis() - lastRotateWarnMs >= 2000) {
      lastRotateWarnMs = millis();
      addLog("❌ [SAFETY HALT] Turning aborted — MPU6050 is DISCONNECTED!");
    }
    return true; // Abort turn immediately
  }

  // ── Compute shortest-path error (-180° to +180°) ───────────────────────────
  float error = targetAngle - currentYaw;
  while (error > 180.0f)
    error -= 360.0f;
  while (error < -180.0f)
    error += 360.0f;
  float absErr = fabsf(error);

  // Persistent ramp, settle, brake, and unwedge state variables
  static unsigned long rampStartMs = 0;
  static int8_t lastTurnDir = 0;
  static unsigned long settleStartMs = 0;
  static bool brakeInitiated = false;

  static bool unwedgeActive = false;
  static unsigned long unwedgeStartMs = 0;
  static unsigned long maxPwmStallTimer = 0;
  static float lastStallObservedYaw = 0.0f;
  static uint8_t unwedgeAttemptCount = 0;

  const unsigned long SETTLE_WINDOW_MS =
      180; // Required continuous in-tolerance dwell time

  // ── ACCEPTANCE & SETTLE WINDOW (absErr <= tolerance) ───────────────────────
  if (absErr <= tolerance) {
    unwedgeActive = false;
    maxPwmStallTimer = 0;
    unwedgeAttemptCount = 0;

    // First cycle entering the tolerance window: start dwell timer and trigger
    // active brake
    if (settleStartMs == 0) {
      settleStartMs = millis();
      if (!brakeInitiated && !motorBrakingActive) {
        brakeMotors(); // Non-blocking 80ms electrical lock
        brakeInitiated = true;
      }
    }

    static unsigned long lastInTolLogMs = 0;
    if (millis() - lastInTolLogMs >= 80) {
      lastInTolLogMs = millis();
      Serial.printf(
          "⏳ [IN TOLERANCE] Target: %6.1f° | Yaw: %6.1f° | Err: %5.1f° (Tol: "
          "±%.1f°) | Dwell: %lu/%lums | PWM: (%d, %d) | Brake: %s\n",
          targetAngle, currentYaw, error, tolerance, (millis() - settleStartMs),
          SETTLE_WINDOW_MS, currentSpeedA, currentSpeedB,
          motorBrakingActive ? "ACTIVE" : "RELEASED");
    }

    // Verify dwell condition: must remain continuously in tolerance for
    // SETTLE_WINDOW_MS AND the non-blocking brake pulse must have fully
    // released.
    if ((millis() - settleStartMs >= SETTLE_WINDOW_MS) && !motorBrakingActive) {
      stopMotors();
      Serial.printf("🎯 [TURN LOCKED] Target: %6.1f° | Final Yaw: %6.1f° | "
                    "Final Err: %5.1f° | Settled in window -> ADVANCING!\n",
                    targetAngle, currentYaw, error);
      settleStartMs = 0;
      brakeInitiated = false;
      rampStartMs = 0;
      lastTurnDir = 0;
      stallBoostLeft = 0;
      stallBoostRight = 0;
      unwedgeActive = false;
      maxPwmStallTimer = 0;
      unwedgeAttemptCount = 0;
      return true; // Turn complete! Safely proceed to next state.
    }

    return false; // Still settling or brake pulse in flight
  }

  // ── OUTSIDE TOLERANCE WINDOW (absErr > tolerance) ──────────────────────────
  settleStartMs = 0;
  brakeInitiated = false;

  int8_t currentTurnDir =
      (error > 0) ? 1
                  : -1; // +1 = Left (increases yaw), -1 = Right (decreases yaw)

  // Reset ramp timer if turn direction changed or new turn started
  if (rampStartMs == 0 || currentTurnDir != lastTurnDir) {
    rampStartMs = millis();
    lastTurnDir = currentTurnDir;
    unwedgeActive = false;
    maxPwmStallTimer = 0;
    lastStallObservedYaw = currentYaw;
    unwedgeAttemptCount = 0;
  }

  // ── ACTIVE BREAKAWAY / UNWEDGE KICK MANEUVER (Non-blocking 180ms) ──────────
  if (unwedgeActive) {
    if (millis() - unwedgeStartMs < 180) {
      return false; // Keep breakaway pulse active
    }
    // Breakaway kick pulse finished! Resume normal closed-loop turning
    unwedgeActive = false;
    maxPwmStallTimer = millis();
    lastStallObservedYaw = currentYaw;
  }

  int baseSpd = (currentTurnDir > 0) ? turnLeftSpeed : turnRightSpeed;

  // Smooth linear far-zone ramp speed: base breakaway PWM + 25 PWM / 1000ms up
  // to 255
  unsigned long elapsedMs = millis() - rampStartMs;
  int rampAdd = (int)((elapsedMs * 25) / 1000);
  int farSpd = constrain(baseSpd + rampAdd, baseSpd, 255);

  int spd = farSpd;

  // Deceleration zone: fixed 30° boundary based on physical chassis stopping
  // distance.
  const float decelBoundary =
      30.0f; // Degrees before target to begin slowing down

  if (absErr < decelBoundary) {
    float decelFactor = constrain(
        (absErr - tolerance) / (decelBoundary - tolerance), 0.0f, 1.0f);
    int maxDecelEntrySpd = min(farSpd, baseSpd + 15);
    spd = baseSpd + (int)(decelFactor * (maxDecelEntrySpd - baseSpd));
  }

  if (currentTurnDir > 0) {
    turnLeft(spd); // Error > 0: Need to increase yaw -> Turn LEFT (CCW)
  } else {
    turnRight(spd); // Error < 0: Need to decrease yaw -> Turn RIGHT (CW)
  }

  int activeBoost = (currentTurnDir > 0) ? stallBoostLeft : stallBoostRight;
  int effectivePwm = currentSpeedA; // both A & B receive effective speed

  // ── DETECT STALL AT MAX POWER & TRIGGER BREAKAWAY KICK ────────────────────
  float yawMovedSinceStallCheck = fabsf(currentYaw - lastStallObservedYaw);

  if (effectivePwm >= 245 || activeBoost >= 36) {
    if (yawMovedSinceStallCheck < 0.8f) {
      if (maxPwmStallTimer == 0) {
        maxPwmStallTimer = millis();
        lastStallObservedYaw = currentYaw;
      } else if (millis() - maxPwmStallTimer >= 1200) {
        // Robot has been stuck at full power for 1.2s without rotating!
        unwedgeActive = true;
        unwedgeStartMs = millis();
        unwedgeAttemptCount++;

        if (unwedgeAttemptCount % 2 == 1) {
          // Swing-Pivot Kick: power the outer wheel forward (255 PWM), let
          // inner wheel coast (0 PWM) Rolling leverage breaks static friction
          // and instantly flips the caster wheel!
          Serial.printf("⚡ [BREAKAWAY KICK #%d] Stalled at 255 PWM for 1.2s! "
                        "Applying SWING-PIVOT pulse (outer wheel forward)...\n",
                        unwedgeAttemptCount);
          if (currentTurnDir > 0) {
            // Turning Left: drive right wheel (Motor B) forward, left wheel
            // coast
            digitalWrite(PIN_IN1, LOW);
            digitalWrite(PIN_IN2, LOW);
            digitalWrite(PIN_IN3, LOW);
            digitalWrite(PIN_IN4, HIGH);
            setMotorSpeeds(0, 255);
          } else {
            // Turning Right: drive left wheel (Motor A) forward, right wheel
            // coast
            digitalWrite(PIN_IN1, LOW);
            digitalWrite(PIN_IN2, HIGH);
            digitalWrite(PIN_IN3, LOW);
            digitalWrite(PIN_IN4, LOW);
            setMotorSpeeds(255, 0);
          }
        } else {
          // Reverse-Nudge Kick: brief reverse (180 PWM) to un-stick any
          // physical binding
          Serial.printf("⚡ [BREAKAWAY KICK #%d] Stalled at 255 PWM for 1.2s! "
                        "Applying REVERSE-NUDGE pulse...\n",
                        unwedgeAttemptCount);
          digitalWrite(PIN_IN1, HIGH);
          digitalWrite(PIN_IN2, LOW);
          digitalWrite(PIN_IN3, HIGH);
          digitalWrite(PIN_IN4, LOW);
          setMotorSpeeds(180, 180);
        }
        return false;
      }
    } else {
      // Yaw changed by > 0.8° -> robot is actively moving! Reset stall timer
      maxPwmStallTimer = millis();
      lastStallObservedYaw = currentYaw;
      unwedgeAttemptCount = 0;
    }
  } else {
    maxPwmStallTimer = millis();
    lastStallObservedYaw = currentYaw;
  }

  static unsigned long lastOutTolLogMs = 0;
  if (millis() - lastOutTolLogMs >= 100) {
    lastOutTolLogMs = millis();
    Serial.printf("🧭 [TURN %-5s] Target: %6.1f° | Yaw: %6.1f° | Err: %5.1f° "
                  "(Tol: ±%.1f°) | Rate: %5.1f°/s | Base: %d | Ramp: +%d | "
                  "StallBoost: +%d | ENA(L): %3d | ENB(R): %3d | Zone: %s\n",
                  (currentTurnDir > 0 ? "LEFT" : "RIGHT"), targetAngle,
                  currentYaw, error, tolerance, currentGyroRateZ, baseSpd,
                  (spd - baseSpd), activeBoost, currentSpeedA, currentSpeedB,
                  (absErr < decelBoundary ? "DECEL" : "FAR_RAMP"));
  }

  return false;
}

// ======================== CAM-GUIDED STATE MACHINE ===========================
#define IDLE_DURATION_MS 2000 // 2 s idle after full 360° scan

enum FacingOrientation : uint8_t {
  ORIENTATION_FRONT = 0,
  ORIENTATION_REAR = 1
};

enum CamGuidedSub {
  CG_IDLE,         // Initial entry / reset
  CG_SCAN_CHECK,   // Facing straight ahead (90°), waiting for ESP32-CAM frame
                   // capture & Gemini response
  CG_SCAN_TURN_90, // Rotate chassis 90° LEFT with MPU6050 to scan next
                   // direction
  CG_TRACKING, // Person detected — moving forward & curve-steering straight to
               // target
  CG_CHECK_OBSTACLE, // Front obstacle detected while tracking — query Gemini
                     // (target vs blocker)
  CG_AVOID_OBSTACLE, // Obstacle in path confirmed blocker — 7-point sweep &
                     // closed-loop angle swerve
  CG_PRESENT_TURN,   // Turn 90° Left upon reaching target to present bin
  CG_WAIT_DROP,      // Wait 5 seconds for waste drop
  CG_RESTORE_TURN,   // Turn 90° Left again to face away from person
  CG_DEPART_DRIVE    // Drive forward away from person for 2 seconds
                     // (accel-verified), then clear target & resume scan
};

CamGuidedSub cgSub = CG_IDLE;
FacingOrientation robotOrientation = ORIENTATION_FRONT;
int currentCycle = 1;
int currentStep = 0;
unsigned long stepStartMs = 0;
unsigned long idleStartMs = 0;
unsigned long cgReverseStart = 0;
unsigned long haltStartMs = 0;
unsigned long cgDepartTimer = 0;
unsigned long cgVerifiedDepartMs = 0;
unsigned long obstacleCheckTimer = 0;
int avoidStep = 0;
unsigned long avoidTimer = 0;
float leftAvoidDist = -1.0f;
float rightAvoidDist = -1.0f;
float cgRotateTarget = 0.0f;
float cgSavedApproachYaw = 0.0f;
char camStateStr[64] = "IDLE";
int8_t camAngleOffset = 0;

// Obstacle Avoidance & Gated Reverse Backup State
unsigned long verifiedBackupMs = 0;
unsigned long backupLastMs = 0;
unsigned long backupStartWallMs = 0;
int cgAvoidRetries = 0;
int cgSweepIdx = 0;
float cgSweepDistances[7] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};
int avoidSubStep = 0;
int classAvoidRetries = 0;
unsigned long avoidSubTimer = 0;
float avoidLeftDist = -1.0f;
float avoidRightDist = -1.0f;
float targetAvoidHeading = 0.0f;

// 7-Point Panoramic Sweep Angles (0° = Right, 90° = Center, 180° = Left)
const int SCAN_ANGLES[7] = {0, 30, 60, 90, 120, 150, 180};
float sweepDistances[7] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};
int currentSweepIdx = 0;

// Evaluates the 7-point sweep and returns the best angle (0 to 180).
// Returns 90 if center is clear, or best angle (0..180) based on clearance and
// minimum deviation. Returns -1 if all directions are blocked.
int evaluateSweepPath(const float dists[7], float thresholdCm) {
  char mapBuf[160];
  snprintf(mapBuf, sizeof(mapBuf),
           "📡 [SWEEP MAP] 0°:%.0fcm | 30°:%.0fcm | 60°:%.0fcm | 90°:%.0fcm | "
           "120°:%.0fcm | 150°:%.0fcm | 180°:%.0fcm",
           dists[0], dists[1], dists[2], dists[3], dists[4], dists[5],
           dists[6]);
  addLog(mapBuf);

  // If dead-ahead (90°) has a valid distance > threshold, continue forward
  if (dists[3] > thresholdCm) {
    return 90;
  }

  // Candidate angle pairs sorted by minimum angular deviation from 90°
  // Pair 0: 120° (Left +30°) vs 60° (Right -30°)
  // Pair 1: 150° (Left +60°) vs 30° (Right -60°)
  // Pair 2: 180° (Left +90°) vs 0°  (Right -90°)
  int candidatePairs[3][2] = {
      {4, 2}, // index 4 = 120°, index 2 = 60°
      {5, 1}, // index 5 = 150°, index 1 = 30°
      {6, 0}  // index 6 = 180°, index 0 = 0°
  };

  for (int pair = 0; pair < 3; pair++) {
    int lIdx = candidatePairs[pair][0];
    int rIdx = candidatePairs[pair][1];

    // When front is blocked by a wall, timeouts (<= 0) at angled positions are
    // specular acoustic reflections off the wall itself, NOT open air. A
    // direction is ONLY considered CLEAR if it returned a genuine valid
    // measurement > thresholdCm.
    bool lClear = (dists[lIdx] > thresholdCm);
    bool rClear = (dists[rIdx] > thresholdCm);

    float lDist = lClear ? dists[lIdx] : 0.0f;
    float rDist = rClear ? dists[rIdx] : 0.0f;

    if (lClear && rClear) {
      // Both sides clear: pick the side with greater clearance depth
      return (lDist >= rDist) ? SCAN_ANGLES[lIdx] : SCAN_ANGLES[rIdx];
    } else if (lClear) {
      return SCAN_ANGLES[lIdx];
    } else if (rClear) {
      return SCAN_ANGLES[rIdx];
    }
  }

  return -1; // All angles blocked by wall/obstacle -> Trigger reverse backup!
}

void processCamGuided() {
  if (currentMode != MODE_CAM_GUIDED)
    return;
  if (camPaused) {
    stopMotors();
    return;
  }

  // Hard Safety Interlock: MPU6050 must be connected
  if (!mpuConnected) {
    stopMotors();
    strncpy(camStateStr, "HALTED: MPU DISCONNECTED", sizeof(camStateStr));
    static unsigned long lastCamMpuWarn = 0;
    if (millis() - lastCamMpuWarn >= 3000) {
      lastCamMpuWarn = millis();
      addLog("🚨 [SAFETY HALT] Camera Mode halted — MPU6050 connection lost!");
    }
    return;
  }

  CamPacket pkt = lastCamPkt;
  bool isNewPacket = newCamPkt;
  newCamPkt = false;

  camAngleOffset = pkt.angleOffset;
  unsigned long now = millis();

  // ── 90° Turn Left to Present Bin to Person ───────────────────────────────
  if (cgSub == CG_PRESENT_TURN) {
    strncpy(camStateStr, "PRESENTING BIN (90° L)", sizeof(camStateStr));
    if (rotateToHeading(cgRotateTarget)) {
      stopMotors();
      haltStartMs = millis();
      addLog("🗑️ Bin presented! Waiting 5.0s for waste disposal...");
      cgSub = CG_WAIT_DROP;
      strncpy(camStateStr, "WAITING FOR DROP (5s)", sizeof(camStateStr));
    }
    return;
  }

  // ── Wait 5s for Waste Disposal ───────────────────────────────────────────
  if (cgSub == CG_WAIT_DROP) {
    stopMotors();
    strncpy(camStateStr, "WAITING FOR DROP (5s)", sizeof(camStateStr));
    if (now - haltStartMs >= 5000 || (isNewPacket && pkt.state == 3)) {
      cgRotateTarget =
          currentYaw +
          90.0f; // Turn 90° LEFT again from current presented bin position
      addLog("⏰ 5s elapsed / drop confirmed! Turning 90° LEFT to face "
             "departure heading...");
      cgSub = CG_RESTORE_TURN;
      strncpy(camStateStr, "TURNING 90° L DEPART", sizeof(camStateStr));
    }
    return;
  }

  // ── 90° Turn Left to Face Away from Person ────────────────────────────────
  if (cgSub == CG_RESTORE_TURN) {
    strncpy(camStateStr, "TURNING 90° L DEPART", sizeof(camStateStr));
    if (rotateToHeading(cgRotateTarget)) {
      stopMotors();
      writeUltrasonicServo(90);
      writeCamServo(CAM_ANGLE_CENTER);
      cgDepartTimer = millis();
      cgVerifiedDepartMs = 0;
      addLog("🚗 90° Left turn locked! Moving forward away from person (2.0s "
             "verified)...");
      cgSub = CG_DEPART_DRIVE;
      strncpy(camStateStr, "DEPARTING (2s)", sizeof(camStateStr));
    }
    return;
  }

  // ── Move Forward Away from Person for 2 Seconds & Reset Target ────────────
  if (cgSub == CG_DEPART_DRIVE) {
    strncpy(camStateStr, "DEPARTING (2s)", sizeof(camStateStr));
    writeUltrasonicServo(90);
    moveForward(defaultDriveSpeed);

    unsigned long dtMs = (cgDepartTimer == 0) ? 0 : (now - cgDepartTimer);
    cgDepartTimer = now;
    bool isMoving = isRobotMovingLinear();
    if (isMoving && dtMs > 0 && dtMs < 500) {
      cgVerifiedDepartMs += dtMs;
    }

    // Obstacle safety check during departure
    if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
      stopMotors();
      addLog("🚧 Obstacle ahead during departure — stopping departure drive "
             "early.");
      cgVerifiedDepartMs = 2000; // Trigger completion
    }

    if (cgVerifiedDepartMs >= 2000) {
      stopMotors();
      cgVerifiedDepartMs = 0;
      cgDepartTimer = 0;
      writeCamServo(CAM_ANGLE_CENTER);
      writeUltrasonicServo(90);

      // CLEAR ALL TARGET DATA & PACKETS so robot starts with a fresh slate
      lastCamPkt.state = 0;
      lastCamPkt.angleOffset = 0;
      newCamPkt = false;
      broadcastMode(4); // Send explicit vision reset (pktType 4) to ESP32-CAM
      delay(10);
      broadcastMode(0); // Sync current mode (CAM_GUIDED)

      cgSub = CG_IDLE;
      strncpy(camStateStr, "IDLE", sizeof(camStateStr));
      addLog("✅ Departure complete (2.0s verified). Target cleared. Starting "
             "fresh 90° scan...");
    }
    return;
  }

  // ── 90° Left Chassis Turn to Next Scan Quadrant ───────────────────────────
  if (cgSub == CG_SCAN_TURN_90) {
    strncpy(camStateStr, "TURNING 90° L NEXT VIEW", sizeof(camStateStr));
    if (rotateToHeading(cgRotateTarget)) {
      stopMotors();
      writeCamServo(CAM_ANGLE_CENTER);
      writeUltrasonicServo(90);
      broadcastMode(
          4); // Command ESP32-CAM to begin fresh capture in this new direction
      delay(10);
      broadcastMode(0);
      stepStartMs = millis();
      cgSub = CG_SCAN_CHECK;
      strncpy(camStateStr, "SCANNING AHEAD (90°)", sizeof(camStateStr));
      char qBuf[80];
      snprintf(qBuf, sizeof(qBuf),
               "✅ 90° Left turn locked (Heading: %.1f°). Capturing frame "
               "ahead at 90°...",
               currentYaw);
      addLog(qBuf);
    }
    return;
  }

  // ── State Machine Processing ──────────────────────────────────────────────
  switch (cgSub) {

  case CG_IDLE:
    stopMotors();
    writeCamServo(CAM_ANGLE_CENTER);
    writeUltrasonicServo(90);
    broadcastMode(4);
    delay(10);
    broadcastMode(0);
    stepStartMs = now;
    cgSub = CG_SCAN_CHECK;
    strncpy(camStateStr, "SCANNING AHEAD (90°)", sizeof(camStateStr));
    addLog("📡 [CAM SCAN] Camera locked at 90° ahead — capturing frame for "
           "Gemini...");
    break;

  case CG_SCAN_CHECK:
    stopMotors();

    // If target detected in front (state 1 = TRACKING)
    if (isNewPacket && pkt.state == 1) {
      addLog("🎯 Target detected directly ahead! Approaching target...");
      cgSub = CG_TRACKING;
      strncpy(camStateStr, "TRACKING TARGET", sizeof(camStateStr));
      curveSteer(pkt.angleOffset);
      return;
    }

    // If no target confirmed (state 4 = SCAN_STEP_DONE / NONE)
    if (isNewPacket && pkt.state == 4) {
      addLog("🔄 No target ahead in this quadrant — turning 90° LEFT to scan "
             "next direction...");
      cgRotateTarget = currentYaw + 90.0f; // Turn 90° Left (+90° CCW)
      cgSub = CG_SCAN_TURN_90;
      strncpy(camStateStr, "TURNING 90° L NEXT VIEW", sizeof(camStateStr));
      return;
    }

    // Guard timeout (14 seconds with no response from camera/network)
    if (now - stepStartMs >= 14000) {
      addLog("⏱️ Scan check timed out — turning 90° LEFT to scan next "
             "direction...");
      cgRotateTarget = currentYaw + 90.0f;
      cgSub = CG_SCAN_TURN_90;
      strncpy(camStateStr, "TURNING 90° L NEXT VIEW", sizeof(camStateStr));
    }
    break;

  case CG_TRACKING:
    strncpy(camStateStr, "TRACKING", sizeof(camStateStr));
    if (pkt.state == 1) {
      // Obstacle detected in path while approaching person -> verify if
      // obstacle is target or blocker
      if (latestDistanceCm > 0 && latestDistanceCm < obstacleThresholdCm) {
        stopMotors();
        writeCamServo(CAM_ANGLE_CENTER);
        writeUltrasonicServo(90);
        cgSub = CG_CHECK_OBSTACLE;
        obstacleCheckTimer = now;
        strncpy(camStateStr, "VERIFYING OBSTACLE", sizeof(camStateStr));
        addLog("🚧 Obstacle at " + String(latestDistanceCm, 1) +
               " cm! Querying Gemini to verify if it is target or blocker...");
        broadcastMode(5); // Trigger camera node state 5
        return;
      }
      curveSteer(pkt.angleOffset);
    } else if (pkt.state == 2) { // INTERACTION (reached person)
      stopMotors();
      writeCamServo(CAM_ANGLE_CENTER);
      writeUltrasonicServo(90);
      cgSavedApproachYaw = currentYaw;
      cgRotateTarget = currentYaw + 90.0f; // Turn 90° Left (+90°)
      addLog("🎯 Target reached! Turning 90° LEFT to present waste bin...");
      cgSub = CG_PRESENT_TURN;
      strncpy(camStateStr, "PRESENTING BIN (90° L)", sizeof(camStateStr));
    } else if (pkt.state == 0) { // Lost target
      addLog("⚠️ Target lost — restarting autonomous scan.");
      stopMotors();
      writeUltrasonicServo(90);
      cgSub = CG_IDLE;
    }
    break;

  case CG_CHECK_OBSTACLE: {
    stopMotors();
    strncpy(camStateStr, "VERIFYING OBSTACLE", sizeof(camStateStr));

    // Handle response from ESP32-CAM (states 6 = IS_TARGET, 7 = IS_OBSTACLE, 2
    // = INTERACTION, 3 = RELEASE)
    if (isNewPacket) {
      if (pkt.state == 6 || pkt.state == 2 || pkt.state == 3) {
        stopMotors();
        writeCamServo(CAM_ANGLE_CENTER);
        writeUltrasonicServo(90);
        cgSavedApproachYaw = currentYaw;
        cgRotateTarget = currentYaw + 90.0f; // Turn 90° Left (+90°)
        addLog("🎯 Gemini confirmed obstacle IS the target! Turning 90° LEFT "
               "to present bin...");
        cgSub = CG_PRESENT_TURN;
        strncpy(camStateStr, "PRESENTING BIN (90° L)", sizeof(camStateStr));
      } else if (pkt.state == 7 || pkt.state == 0) {
        addLog("🚧 Gemini confirmed obstacle is an UNRELATED BLOCKER. Starting "
               "180° panoramic scan...");
        avoidStep = 0;
        avoidTimer = now;
        writeUltrasonicServo(SCAN_ANGLES[0]); // Start sweep at 0°
        cgSub = CG_AVOID_OBSTACLE;
        strncpy(camStateStr, "AVOIDING OBSTACLE", sizeof(camStateStr));
      }
    }

    // Timeout fallback after 10 seconds if no Gemini answer arrives
    if (now - obstacleCheckTimer >= 10000) {
      addLog(
          "⏱️ Gemini verification timed out — starting 180° panoramic scan...");
      avoidStep = 0;
      avoidTimer = now;
      writeUltrasonicServo(SCAN_ANGLES[0]);
      cgSub = CG_AVOID_OBSTACLE;
    }
    break;
  }

  case CG_AVOID_OBSTACLE: {
    strncpy(camStateStr, "AVOIDING OBSTACLE", sizeof(camStateStr));

    switch (avoidStep) {
    // Step 0: 7-Point Panoramic Radar Sweep (0° -> 180°) with 200ms dwell &
    // 5-ping max filter
    case 0:
      if (cgSweepIdx < 7) {
        writeUltrasonicServo(SCAN_ANGLES[cgSweepIdx]);
        unsigned long requiredDwell =
            (cgSweepIdx == 0) ? 300 : 200; // 300ms for initial 0° repositioning
        if (now - avoidTimer >= requiredDwell) {
          cgSweepDistances[cgSweepIdx] = readUltrasonicMaxFilter(5, 15);
          cgSweepIdx++;
          avoidTimer = now;
        }
      } else {
        // Sweep complete! Immediately return sensor to 90°
        writeUltrasonicServo(90);
        cgSweepIdx = 0;
        avoidTimer = now;
        avoidStep = 1;
      }
      break;

    // Step 1: Decision & Target Calculation
    case 1:
      if (now - avoidTimer >= 150) {
        int bestAngle =
            evaluateSweepPath(cgSweepDistances, obstacleThresholdCm);
        if (bestAngle == 90) {
          addLog("✅ Path straight ahead is clear — resuming camera tracking.");
          writeUltrasonicServo(90);
          cgAvoidRetries = 0;
          cgSub = CG_TRACKING;
        } else if (bestAngle >= 0) {
          float angleOffset = (float)(bestAngle - 90);
          cgRotateTarget = currentYaw + angleOffset;
          char planBuf[96];
          snprintf(planBuf, sizeof(planBuf),
                   "💡 Swerving to %d° corridor (Offset: %+.0f°) -> Target "
                   "Heading: %.1f°",
                   bestAngle, angleOffset, cgRotateTarget);
          addLog(planBuf);
          cgAvoidRetries = 0;
          avoidStep = 2; // Advance to closed-loop turn
        } else {
          // All 7 angles blocked
          cgAvoidRetries++;
          char retryBuf[80];
          snprintf(retryBuf, sizeof(retryBuf),
                   "⚠️ All 7 sweep angles blocked (Attempt %d/5)...",
                   cgAvoidRetries);
          addLog(retryBuf);

          if (cgAvoidRetries >= 5) {
            addLog("🚨 DEADLOCK: 5 failed attempts! Turning 180° LEFT and "
                   "restarting scan...");
            cgRotateTarget = currentYaw + 180.0f;
            avoidStep = 5; // Deadlock 180° turn sub-step
          } else {
            addLog(
                "⚠️ Reversing 1000ms (accel-verified) to open up clearance...");
            verifiedBackupMs = 0;
            backupLastMs = millis();
            backupStartWallMs = millis();
            avoidStep = 4; // Accel-gated backup sub-step
          }
        }
      }
      break;

    // Step 2: Closed-Loop Gyro Heading Turn to Swerve Angle
    case 2:
      if (rotateToHeading(cgRotateTarget)) {
        stopMotors();
        writeUltrasonicServo(90); // Guarantee sensor looking forward
        avoidTimer = millis();
        avoidStep = 3;
      }
      break;

    // Step 3: Forward Bypass & Real-time Obstacle Safety Check
    case 3: {
      writeUltrasonicServo(90); // Keep looking dead ahead

      // Real-time obstacle check while on bypass movement!
      float curDist = readUltrasonic();
      if (curDist > 0 && curDist <= obstacleThresholdCm) {
        stopMotors();
        char obsWarn[96];
        snprintf(obsWarn, sizeof(obsWarn),
                 "🚨 Secondary obstacle ahead during bypass (%.1f cm)! "
                 "Reversing to clear...",
                 curDist);
        addLog(obsWarn);
        cgAvoidRetries++;
        if (cgAvoidRetries >= 5) {
          addLog("🚨 DEADLOCK: 5 failed attempts! Turning 180° LEFT and "
                 "restarting scan...");
          cgRotateTarget = currentYaw + 180.0f;
          avoidStep = 5; // 180° escape
        } else {
          verifiedBackupMs = 0;
          backupLastMs = millis();
          backupStartWallMs = millis();
          avoidStep = 4; // Back up 1000ms
        }
        break;
      }

      moveForward(defaultDriveSpeed);
      if (now - avoidTimer >= 1000) {
        stopMotors();
        writeUltrasonicServo(90);
        addLog("✅ Obstacle bypassed — resuming camera tracking.");
        cgSub = CG_TRACKING;
      }
      break;
    }

    // Step 4: Accelerometer-Gated Reverse Backup (1000ms verified motion)
    case 4: {
      moveBackward(defaultDriveSpeed);
      unsigned long dtMs = (backupLastMs == 0) ? 0 : (now - backupLastMs);
      backupLastMs = now;

      bool moving = isRobotMovingLinear();
      if (moving && dtMs > 0 && dtMs < 500) {
        verifiedBackupMs += dtMs;
      }

      static unsigned long lastCgLogMs = 0;
      if (now - lastCgLogMs >= 250) {
        lastCgLogMs = now;
        Serial.printf("⚡ [CAM BACKUP] Motion: %s (Jerk: %.3f) | Verified: "
                      "%lu/1000ms | PWM: %d\n",
                      moving ? "MOVING" : "STALLED/SLIP", currentAccelJerk,
                      verifiedBackupMs, currentSpeedA);
      }

      if (verifiedBackupMs >= 1000 || (now - backupStartWallMs >= 3500)) {
        stopMotors();
        writeUltrasonicServo(90);
        verifiedBackupMs = 0;
        backupLastMs = 0;
        avoidStep = 0; // Re-scan
        avoidTimer = millis();
      }
      break;
    }

    // Step 5: Deadlock Escape 180° Turn & Fresh Scan
    case 5:
      if (rotateToHeading(cgRotateTarget)) {
        stopMotors();
        writeUltrasonicServo(90);
        writeCamServo(CAM_ANGLE_CENTER);
        cgAvoidRetries = 0;
        broadcastMode(4); // Send explicit vision reset (pktType 4) to ESP32-CAM
        delay(10);
        broadcastMode(0); // Sync current mode
        robotOrientation = ORIENTATION_FRONT;
        currentCycle = 1;
        currentStep = 0;
        cgSub = CG_IDLE;
        strncpy(camStateStr, "IDLE", sizeof(camStateStr));
        addLog("✅ 180° Turn complete — starting fresh 360° scan for new "
               "targets...");
      }
      break;
    }
    break;
  }

  default:
    break;
  }
}

// ======================== MANUAL MODE CONTINUOUS DRIVE & AVOIDANCE
// ============
void processManualMode() {
  if (currentMode != MODE_MANUAL)
    return;

  unsigned long now = millis();

  switch (manualDriveState) {

  case MANUAL_IDLE:
    // Stopped
    break;

  case MANUAL_DRIVE_FORWARD: {
    moveForward(defaultDriveSpeed);
    writeUltrasonicServo(90);

    // Active obstacle check while cruising forward
    if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
      stopMotors();
      char obsBuf[96];
      snprintf(obsBuf, sizeof(obsBuf),
               "🚨 [MANUAL] Obstacle at %.1f cm! Initiating avoidance sweep...",
               latestDistanceCm);
      addLog(obsBuf);
      manualResumeState = MANUAL_DRIVE_FORWARD;
      avoidSubStep = 0;
      avoidSubTimer = now;
      classAvoidRetries = 0;
      manualDriveState = MANUAL_OBSTACLE_AVOID;
    }
    break;
  }

  case MANUAL_DRIVE_BACKWARD: {
    moveBackward(defaultDriveSpeed);
    // Keep moving backward continuously until STOP is pressed
    break;
  }

  case MANUAL_DRIVE_CIRCLE_LEFT: {
    circleLeft(turnLeftSpeed);
    writeUltrasonicServo(90);

    // Active obstacle check while circling
    if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
      stopMotors();
      char obsBuf[96];
      snprintf(obsBuf, sizeof(obsBuf),
               "🚨 [MANUAL] Obstacle at %.1f cm while circling LEFT! "
               "Initiating avoidance...",
               latestDistanceCm);
      addLog(obsBuf);
      manualResumeState = MANUAL_DRIVE_CIRCLE_LEFT;
      avoidSubStep = 0;
      avoidSubTimer = now;
      classAvoidRetries = 0;
      manualDriveState = MANUAL_OBSTACLE_AVOID;
    }
    break;
  }

  case MANUAL_DRIVE_CIRCLE_RIGHT: {
    circleRight(turnRightSpeed);
    writeUltrasonicServo(90);

    // Active obstacle check while circling
    if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
      stopMotors();
      char obsBuf[96];
      snprintf(obsBuf, sizeof(obsBuf),
               "🚨 [MANUAL] Obstacle at %.1f cm while circling RIGHT! "
               "Initiating avoidance...",
               latestDistanceCm);
      addLog(obsBuf);
      manualResumeState = MANUAL_DRIVE_CIRCLE_RIGHT;
      avoidSubStep = 0;
      avoidSubTimer = now;
      classAvoidRetries = 0;
      manualDriveState = MANUAL_OBSTACLE_AVOID;
    }
    break;
  }

  case MANUAL_OBSTACLE_AVOID: {
    switch (avoidSubStep) {
    // Sub-step 0: 7-Point Panoramic Radar Sweep (0° -> 180°) with 200ms dwell &
    // 5-ping max filter
    case 0:
      if (currentSweepIdx < 7) {
        writeUltrasonicServo(SCAN_ANGLES[currentSweepIdx]);
        unsigned long requiredDwell = (currentSweepIdx == 0) ? 300 : 200;
        if (now - avoidSubTimer >= requiredDwell) {
          sweepDistances[currentSweepIdx] = readUltrasonicMaxFilter(5, 15);
          currentSweepIdx++;
          avoidSubTimer = now;
        }
      } else {
        writeUltrasonicServo(90);
        currentSweepIdx = 0;
        avoidSubTimer = now;
        avoidSubStep = 1;
      }
      break;

    // Sub-step 1: Decision & Target Calculation
    case 1:
      if (now - avoidSubTimer >= 150) {
        int bestAngle = evaluateSweepPath(sweepDistances, obstacleThresholdCm);
        if (bestAngle == 90) {
          addLog("✅ Path straight ahead is clear — resuming manual drive.");
          writeUltrasonicServo(90);
          classAvoidRetries = 0;
          manualDriveState = manualResumeState;
        } else if (bestAngle >= 0) {
          float angleOffset = (float)(bestAngle - 90);
          targetAvoidHeading = currentYaw + angleOffset;
          char planBuf[96];
          snprintf(
              planBuf, sizeof(planBuf),
              "💡 Swerving to %d° corridor (Offset: %+.0f°) -> Heading: %.1f°",
              bestAngle, angleOffset, targetAvoidHeading);
          addLog(planBuf);
          classAvoidRetries = 0;
          avoidSubStep = 2;
        } else {
          // All 7 angles blocked
          classAvoidRetries++;
          char retryBuf[80];
          snprintf(retryBuf, sizeof(retryBuf),
                   "⚠️ All 7 sweep angles blocked (Attempt %d/5)...",
                   classAvoidRetries);
          addLog(retryBuf);

          if (classAvoidRetries >= 5) {
            addLog("🚨 DEADLOCK: 5 failed attempts! Stopping motors.");
            stopMotors();
            writeUltrasonicServo(90);
            manualDriveState = MANUAL_IDLE;
          } else {
            addLog(
                "⚠️ Reversing 1000ms (accel-verified) to open up clearance...");
            verifiedBackupMs = 0;
            backupLastMs = millis();
            backupStartWallMs = millis();
            avoidSubStep = 5; // Backup sub-step
          }
        }
      }
      break;

    // Sub-step 2: Closed-Loop Gyro Heading Turn to Swerve Angle
    case 2:
      if (rotateToHeading(targetAvoidHeading)) {
        stopMotors();
        writeUltrasonicServo(90);
        avoidSubTimer = millis();
        avoidSubStep = 3;
      }
      break;

    // Sub-step 3: Forward Bypass Drive (1.2s) & Real-time Obstacle Safety Check
    case 3: {
      writeUltrasonicServo(90);
      float curDist = readUltrasonic();
      if (curDist > 0 && curDist <= obstacleThresholdCm) {
        stopMotors();
        char obsWarn[96];
        snprintf(
            obsWarn, sizeof(obsWarn),
            "🚨 Secondary obstacle ahead during bypass (%.1f cm)! Reversing...",
            curDist);
        addLog(obsWarn);
        classAvoidRetries++;
        if (classAvoidRetries >= 5) {
          addLog("🚨 DEADLOCK: 5 failed attempts! Stopping motors.");
          stopMotors();
          manualDriveState = MANUAL_IDLE;
        } else {
          verifiedBackupMs = 0;
          backupLastMs = millis();
          backupStartWallMs = millis();
          avoidSubStep = 5;
        }
        break;
      }

      moveForward(defaultDriveSpeed);
      if (now - avoidSubTimer >= 1200) {
        stopMotors();
        writeUltrasonicServo(90);
        addLog("✅ Obstacle bypassed! Resuming manual drive mode...");
        manualDriveState = manualResumeState;
      }
      break;
    }

    // Sub-step 5: Accelerometer-Gated Reverse Backup (1000ms verified motion)
    case 5: {
      moveBackward(defaultDriveSpeed);
      unsigned long dtMs = (backupLastMs == 0) ? 0 : (now - backupLastMs);
      backupLastMs = now;

      bool moving = isRobotMovingLinear();
      if (moving && dtMs > 0 && dtMs < 500) {
        verifiedBackupMs += dtMs;
      }

      if (verifiedBackupMs >= 1000 || (now - backupStartWallMs >= 3500)) {
        stopMotors();
        writeUltrasonicServo(90);
        verifiedBackupMs = 0;
        backupLastMs = 0;
        avoidSubStep = 0; // Retry sweep
        avoidSubTimer = millis();
      }
      break;
    }
    }
    break;
  }
  }
}

// ======================== 3-CLASSROOM PATROL AUTONOMOUS ======================
enum ClassStep {
  CLASS_INIT,
  CLASS_CORRIDOR_DRIVE,
  CLASS_TURN_ROOM,
  CLASS_ENTER_ROOM,
  CLASS_PRESENT_TURN,
  CLASS_WAIT_DROP,
  CLASS_RESTORE_TURN,
  CLASS_EXIT_ROOM,
  CLASS_TURN_CORRIDOR,
  CLASS_RETURN_TURN,
  CLASS_RETURN_DRIVE,
  CLASS_RETURN_ALIGN,
  CLASS_MISSION_DONE,
  CLASS_OBSTACLE_AVOID
};

ClassStep classStep = CLASS_INIT;
int currentClassroom = 1;
float corridorHeading = 0.0f;
float targetHeading = 0.0f;
unsigned long classStepTimer = 0;
unsigned long targetDurationMs = 0;
unsigned long remainingMoveMs = 0;
unsigned long accumulatedMoveMs =
    0; // Accel-gated verified forward/reverse motion duration
unsigned long lastClassMotionTickMs = 0; // Timestamp for motion delta-time

ClassStep returnAfterAvoidStep = CLASS_CORRIDOR_DRIVE;
float returnAfterAvoidHeading = 0.0f;
char autoStatusStr[32] = "IDLE";

void processClassroomAuto() {
  if (currentMode != MODE_AUTONOMOUS)
    return;

  // Hard Safety Interlock: MPU6050 must be connected
  if (!mpuConnected) {
    stopMotors();
    strncpy(autoStatusStr, "HALTED: MPU DISCONNECTED", sizeof(autoStatusStr));
    static unsigned long lastClassMpuWarn = 0;
    if (millis() - lastClassMpuWarn >= 3000) {
      lastClassMpuWarn = millis();
      addLog("🚨 [SAFETY HALT] 3-Classroom Patrol halted — MPU6050 connection "
             "lost!");
    }
    classStep = CLASS_MISSION_DONE;
    return;
  }

  unsigned long now = millis();

  // Telemetry stream for 3-Classroom Autonomous Mode
  static unsigned long lastClassLogMs = 0;
  if (now - lastClassLogMs >= 200) {
    lastClassLogMs = now;
    Serial.printf(
        "🏫 [CLASS AUTO] State: %-26s | Yaw: %6.1f° | Target: %6.1f° | "
        "CorridorBase: %6.1f° | Dist: %5.1f cm | Motor: %-8s (%d, %d)\n",
        autoStatusStr, currentYaw, targetHeading, corridorHeading,
        latestDistanceCm, currentMotorState, currentSpeedA, currentSpeedB);
  }

  switch (classStep) {

  case CLASS_INIT:
    currentClassroom = 1;
    corridorHeading = currentYaw; // Baseline corridor heading
    targetHeading = corridorHeading;
    targetDurationMs = 2000; // 2 seconds forward
    accumulatedMoveMs = 0;
    lastClassMotionTickMs = millis();
    classAvoidRetries = 0;
    writeUltrasonicServo(90);
    writeCamServo(CAM_ANGLE_CENTER);
    addLog("🏫 Starting 3-Classroom Patrol Mission (Classroom 1/3)...");
    addLog("🚗 [Room 1/3] Moving forward along corridor (2.0s verified)...");
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM 1/3 — CORRIDOR (2s)");
    classStep = CLASS_CORRIDOR_DRIVE;
    break;

  // 1. Move Forward along corridor (2s verified motion)
  case CLASS_CORRIDOR_DRIVE: {
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — CORRIDOR (2s)",
             currentClassroom);
    moveForward(defaultDriveSpeed);

    unsigned long dtMs =
        (lastClassMotionTickMs == 0) ? 0 : (now - lastClassMotionTickMs);
    lastClassMotionTickMs = now;
    bool isMoving = isRobotMovingLinear();
    if (isMoving && dtMs > 0 && dtMs < 500) {
      accumulatedMoveMs += dtMs;
    }

    // Obstacle Check
    if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
      stopMotors();
      remainingMoveMs = (accumulatedMoveMs < targetDurationMs)
                            ? (targetDurationMs - accumulatedMoveMs)
                            : 100;
      returnAfterAvoidStep = CLASS_CORRIDOR_DRIVE;
      returnAfterAvoidHeading = corridorHeading;
      avoidSubStep = 0;
      avoidSubTimer = now;
      classAvoidRetries = 0;
      addLog("🚨 Obstacle in corridor — checking clearance...");
      classStep = CLASS_OBSTACLE_AVOID;
      break;
    }

    if (accumulatedMoveMs >= targetDurationMs) {
      stopMotors();
      accumulatedMoveMs = 0;
      lastClassMotionTickMs = millis();
      targetHeading =
          corridorHeading + 90.0f; // Turn LEFT 90° into room (+90° CCW)
      addLog("🚪 At classroom entrance. Turning LEFT 90° into room...");
      snprintf(autoStatusStr, sizeof(autoStatusStr),
               "ROOM %d/3 — TURN LEFT 90°", currentClassroom);
      classStep = CLASS_TURN_ROOM;
    }
    break;
  }

  // 2. Turn Left 90° into Classroom
  case CLASS_TURN_ROOM:
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — TURN LEFT 90°",
             currentClassroom);
    if (rotateToHeading(targetHeading)) {
      addLog("➡️ Heading locked! Moving forward into room (2.0s verified)...");
      targetDurationMs = 2000;
      accumulatedMoveMs = 0;
      lastClassMotionTickMs = millis();
      snprintf(autoStatusStr, sizeof(autoStatusStr),
               "ROOM %d/3 — ENTERING (2s)", currentClassroom);
      classStep = CLASS_ENTER_ROOM;
    }
    break;

  // 3. Move Forward into Classroom (2s verified motion)
  case CLASS_ENTER_ROOM: {
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — ENTERING (2s)",
             currentClassroom);
    moveForward(defaultDriveSpeed);

    unsigned long dtMs =
        (lastClassMotionTickMs == 0) ? 0 : (now - lastClassMotionTickMs);
    lastClassMotionTickMs = now;
    bool isMoving = isRobotMovingLinear();
    if (isMoving && dtMs > 0 && dtMs < 500) {
      accumulatedMoveMs += dtMs;
    }

    // Obstacle Check
    if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
      stopMotors();
      remainingMoveMs = (accumulatedMoveMs < targetDurationMs)
                            ? (targetDurationMs - accumulatedMoveMs)
                            : 100;
      returnAfterAvoidStep = CLASS_ENTER_ROOM;
      returnAfterAvoidHeading = targetHeading;
      avoidSubStep = 0;
      avoidSubTimer = now;
      classAvoidRetries = 0;
      addLog("🚨 Obstacle inside classroom!");
      classStep = CLASS_OBSTACLE_AVOID;
      break;
    }

    if (accumulatedMoveMs >= targetDurationMs) {
      stopMotors();
      accumulatedMoveMs = 0;
      lastClassMotionTickMs = millis();
      targetHeading =
          corridorHeading + 180.0f; // Turn 90° LEFT inside classroom (+90° room
                                    // entry + 90° = +180°)
      addLog("🚪 Inside classroom. Turning 90° LEFT to present waste bin...");
      snprintf(autoStatusStr, sizeof(autoStatusStr),
               "ROOM %d/3 — PRESENT BIN (90° L)", currentClassroom);
      classStep = CLASS_PRESENT_TURN;
    }
    break;
  }

  // 4. Turn 90° Left to Present Bin in Classroom
  case CLASS_PRESENT_TURN:
    snprintf(autoStatusStr, sizeof(autoStatusStr),
             "ROOM %d/3 — PRESENT BIN (90° L)", currentClassroom);
    if (rotateToHeading(targetHeading)) {
      stopMotors();
      classStepTimer = millis();
      addLog("🛑 Bin presented! Waiting 5.0s for waste disposal...");
      snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — WAITING (5s)",
               currentClassroom);
      classStep = CLASS_WAIT_DROP;
    }
    break;

  // 5. Wait 5s in Classroom for Waste Drop
  case CLASS_WAIT_DROP:
    stopMotors();
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — WAITING (5s)",
             currentClassroom);
    if (now - classStepTimer >= 5000) {
      targetHeading = corridorHeading +
                      90.0f; // Turn 90° RIGHT back to room entry/exit heading
      addLog("⏰ 5s elapsed! Turning 90° RIGHT to face room exit...");
      snprintf(autoStatusStr, sizeof(autoStatusStr),
               "ROOM %d/3 — RESTORE HEADING", currentClassroom);
      classStep = CLASS_RESTORE_TURN;
    }
    break;

  // 6. Turn 90° Right back to Exit Heading
  case CLASS_RESTORE_TURN:
    snprintf(autoStatusStr, sizeof(autoStatusStr),
             "ROOM %d/3 — RESTORE HEADING", currentClassroom);
    if (rotateToHeading(targetHeading)) {
      stopMotors();
      targetDurationMs = 2000;
      accumulatedMoveMs = 0;
      lastClassMotionTickMs = millis();
      addLog("⬅️ Collection complete! Reversing out of room (2.0s verified)...");
      snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — EXITING (2s)",
               currentClassroom);
      classStep = CLASS_EXIT_ROOM;
    }
    break;

  // 7. Move Backward out of Classroom (2s verified motion)
  case CLASS_EXIT_ROOM: {
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — EXITING (2s)",
             currentClassroom);
    moveBackward(defaultDriveSpeed);

    unsigned long dtMs =
        (lastClassMotionTickMs == 0) ? 0 : (now - lastClassMotionTickMs);
    lastClassMotionTickMs = now;
    bool isMoving = isRobotMovingLinear();
    if (isMoving && dtMs > 0 && dtMs < 500) {
      accumulatedMoveMs += dtMs;
    }

    if (accumulatedMoveMs >= targetDurationMs) {
      stopMotors();
      accumulatedMoveMs = 0;
      lastClassMotionTickMs = millis();
      targetHeading =
          corridorHeading; // Turn back right 90° to corridor baseline
      addLog("🔄 Reached corridor. Turning RIGHT 90° to face path...");
      snprintf(autoStatusStr, sizeof(autoStatusStr),
               "ROOM %d/3 — TURN RIGHT 90°", currentClassroom);
      classStep = CLASS_TURN_CORRIDOR;
    }
    break;
  }

  // 8. Turn Right 90° back to Corridor Heading
  case CLASS_TURN_CORRIDOR:
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — TURN RIGHT 90°",
             currentClassroom);
    if (rotateToHeading(targetHeading)) {
      addLog("✅ Classroom visit completed!");
      currentClassroom++;

      if (currentClassroom <= 3) {
        targetDurationMs = 2000; // Next classroom forward 2s
        accumulatedMoveMs = 0;
        lastClassMotionTickMs = millis();
        addLog("🏫 Moving forward to next classroom (2.0s verified)...");
        snprintf(autoStatusStr, sizeof(autoStatusStr),
                 "ROOM %d/3 — CORRIDOR (2s)", currentClassroom);
        classStep = CLASS_CORRIDOR_DRIVE;
      } else {
        targetHeading = corridorHeading + 180.0f; // Turn 180° for return
        addLog("🏁 All 3 classrooms visited! Turning 180° for Return Home "
               "Routine...");
        strncpy(autoStatusStr, "TURNING 180° RETURN", sizeof(autoStatusStr));
        classStep = CLASS_RETURN_TURN;
      }
    }
    break;

  // 7. Turn 180° to Return Home
  case CLASS_RETURN_TURN:
    strncpy(autoStatusStr, "TURNING 180° RETURN", sizeof(autoStatusStr));
    if (rotateToHeading(targetHeading)) {
      targetDurationMs = 6000; // 6 seconds return drive
      accumulatedMoveMs = 0;
      lastClassMotionTickMs = millis();
      addLog(
          "🚗 Returning home along corridor (Moving forward 6.0s verified)...");
      strncpy(autoStatusStr, "RETURNING HOME (6s)", sizeof(autoStatusStr));
      classStep = CLASS_RETURN_DRIVE;
    }
    break;

  // 8. Return Drive (6s verified motion)
  case CLASS_RETURN_DRIVE: {
    strncpy(autoStatusStr, "RETURNING HOME (6s)", sizeof(autoStatusStr));
    moveForward(defaultDriveSpeed);

    unsigned long dtMs =
        (lastClassMotionTickMs == 0) ? 0 : (now - lastClassMotionTickMs);
    lastClassMotionTickMs = now;
    bool isMoving = isRobotMovingLinear();
    if (isMoving && dtMs > 0 && dtMs < 500) {
      accumulatedMoveMs += dtMs;
    }

    // Obstacle Check
    if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
      stopMotors();
      remainingMoveMs = (accumulatedMoveMs < targetDurationMs)
                            ? (targetDurationMs - accumulatedMoveMs)
                            : 100;
      returnAfterAvoidStep = CLASS_RETURN_DRIVE;
      returnAfterAvoidHeading = targetHeading;
      avoidSubStep = 0;
      avoidSubTimer = now;
      classAvoidRetries = 0;
      addLog("🚨 Obstacle on return path!");
      classStep = CLASS_OBSTACLE_AVOID;
      break;
    }

    if (accumulatedMoveMs >= targetDurationMs) {
      stopMotors();
      accumulatedMoveMs = 0;
      lastClassMotionTickMs = millis();
      targetHeading =
          corridorHeading; // Turn 180° back to original home orientation
      addLog("🏠 Reached home station. Aligning to original orientation...");
      strncpy(autoStatusStr, "ALIGNING HOME", sizeof(autoStatusStr));
      classStep = CLASS_RETURN_ALIGN;
    }
    break;
  }

  // 9. Align to Home Heading
  case CLASS_RETURN_ALIGN:
    strncpy(autoStatusStr, "ALIGNING HOME", sizeof(autoStatusStr));
    if (rotateToHeading(targetHeading)) {
      stopMotors();
      addLog("🎉 3-Classroom Patrol Mission COMPLETED successfully!");
      strncpy(autoStatusStr, "MISSION COMPLETE", sizeof(autoStatusStr));
      classStep = CLASS_MISSION_DONE;
    }
    break;

  case CLASS_MISSION_DONE:
    stopMotors();
    break;

  // 10. Autonomous Obstacle Avoidance Routine
  case CLASS_OBSTACLE_AVOID:
    strncpy(autoStatusStr, "OBSTACLE AVOIDANCE", sizeof(autoStatusStr));
    switch (avoidSubStep) {

    // Sub-step 0: Perform 7-Point Panoramic Radar Sweep (0° -> 180°) with 200ms
    // dwell & 5-ping max filter
    case 0:
      if (currentSweepIdx < 7) {
        writeUltrasonicServo(SCAN_ANGLES[currentSweepIdx]);
        unsigned long requiredDwell =
            (currentSweepIdx == 0) ? 300
                                   : 200; // 300ms for initial 0° repositioning
        if (now - avoidSubTimer >= requiredDwell) {
          sweepDistances[currentSweepIdx] = readUltrasonicMaxFilter(5, 15);
          currentSweepIdx++;
          avoidSubTimer = now;
        }
      } else {
        // Full sweep completed! Immediately re-center servo to 90°
        writeUltrasonicServo(90);
        currentSweepIdx = 0;
        avoidSubTimer = now;
        avoidSubStep = 1; // Advance to decision
      }
      break;

    // Sub-step 1: Decision & Target Calculation
    case 1:
      if (now - avoidSubTimer >= 150) {
        int bestAngle = evaluateSweepPath(sweepDistances, obstacleThresholdCm);
        if (bestAngle == 90) {
          addLog("✅ Path straight ahead is now clear — resuming patrol.");
          writeUltrasonicServo(90);
          classStepTimer = millis() - (targetDurationMs - remainingMoveMs);
          classAvoidRetries = 0;
          classStep = returnAfterAvoidStep;
        } else if (bestAngle >= 0) {
          float angleOffset =
              (float)(bestAngle - 90); // e.g. 120° -> +30°, 60° -> -30°
          targetAvoidHeading = currentYaw + angleOffset;
          char planBuf[96];
          snprintf(planBuf, sizeof(planBuf),
                   "💡 Swerving to %d° corridor (Offset: %+.0f°) -> Target "
                   "Heading: %.1f°",
                   bestAngle, angleOffset, targetAvoidHeading);
          addLog(planBuf);
          classAvoidRetries = 0;
          avoidSubStep = 2; // Advance to closed-loop angle turn
        } else {
          // All 7 angles blocked
          classAvoidRetries++;
          char retryBuf[80];
          snprintf(retryBuf, sizeof(retryBuf),
                   "⚠️ All 7 sweep angles blocked (Attempt %d/5)...",
                   classAvoidRetries);
          addLog(retryBuf);

          if (classAvoidRetries >= 5) {
            addLog("🚨 DEADLOCK: 5 failed attempts! Turning 180° LEFT and "
                   "pausing mission...");
            targetAvoidHeading = currentYaw + 180.0f;
            avoidSubStep = 6; // Deadlock 180° turn sub-step
          } else {
            addLog("⚠️ Path completely blocked — reversing 1000ms "
                   "(accel-verified) to re-scan...");
            verifiedBackupMs = 0;
            backupLastMs = millis();
            backupStartWallMs = millis();
            avoidSubStep = 5; // Accel-gated backup sub-step
          }
        }
      }
      break;

    // Sub-step 2: Closed-Loop Gyro Heading Turn to Swerve Angle
    case 2:
      if (rotateToHeading(targetAvoidHeading)) {
        stopMotors();
        writeUltrasonicServo(90); // Guarantee sensor is looking straight ahead
        avoidSubTimer = millis();
        avoidSubStep = 3; // Advance to bypass drive
      }
      break;

    // Sub-step 3: Forward Bypass Drive (1.2s) & Real-time Obstacle Safety Check
    case 3: {
      writeUltrasonicServo(90); // Keep looking dead ahead

      // Real-time obstacle check while on bypass movement before re-alignment!
      float curDist = readUltrasonic();
      if (curDist > 0 && curDist <= obstacleThresholdCm) {
        stopMotors();
        char obsWarn[96];
        snprintf(obsWarn, sizeof(obsWarn),
                 "🚨 Secondary obstacle ahead during bypass drive (%.1f cm)! "
                 "Reversing to clear...",
                 curDist);
        addLog(obsWarn);
        classAvoidRetries++;
        if (classAvoidRetries >= 5) {
          addLog("🚨 DEADLOCK: 5 failed attempts! Turning 180° LEFT and "
                 "pausing mission...");
          targetAvoidHeading = currentYaw + 180.0f;
          avoidSubStep = 6; // 180° escape & pause
        } else {
          verifiedBackupMs = 0;
          backupLastMs = millis();
          backupStartWallMs = millis();
          avoidSubStep = 5; // Back up 1000ms
        }
        break;
      }

      moveForward(defaultDriveSpeed);
      if (now - avoidSubTimer >= 1200) {
        stopMotors();
        writeUltrasonicServo(90);
        addLog("🔄 Obstacle bypassed! Re-aligning to mission heading...");
        avoidSubStep = 4; // Advance to re-alignment
      }
      break;
    }

    // Sub-step 4: Re-Align to Original Mission Heading
    case 4:
      writeUltrasonicServo(90);
      if (rotateToHeading(returnAfterAvoidHeading)) {
        stopMotors();
        writeUltrasonicServo(90);
        addLog("✅ Re-aligned to mission path. Resuming drive...");
        accumulatedMoveMs = (remainingMoveMs < targetDurationMs)
                                ? (targetDurationMs - remainingMoveMs)
                                : 0;
        lastClassMotionTickMs = millis();
        classStep = returnAfterAvoidStep;
      }
      break;

    // Sub-step 5: Accelerometer-Gated Reverse Backup (1000ms verified motion)
    case 5: {
      moveBackward(defaultDriveSpeed);
      unsigned long dtMs = (backupLastMs == 0) ? 0 : (now - backupLastMs);
      backupLastMs = now;

      bool moving = isRobotMovingLinear();
      if (moving && dtMs > 0 && dtMs < 500) {
        verifiedBackupMs += dtMs;
      }

      static unsigned long lastClassLogMs = 0;
      if (now - lastClassLogMs >= 250) {
        lastClassLogMs = now;
        Serial.printf("⚡ [PATROL BACKUP] Motion: %s (Jerk: %.3f) | Verified: "
                      "%lu/1000ms | PWM: %d\n",
                      moving ? "MOVING" : "STALLED/SLIP", currentAccelJerk,
                      verifiedBackupMs, currentSpeedA);
      }

      if (verifiedBackupMs >= 1000 || (now - backupStartWallMs >= 3500)) {
        stopMotors();
        writeUltrasonicServo(90);
        verifiedBackupMs = 0;
        backupLastMs = 0;
        avoidSubStep = 0; // Retry sweep
        avoidSubTimer = millis();
      }
      break;
    }

    // Sub-step 6: Deadlock Escape 180° Turn & Pause Mission
    case 6:
      if (rotateToHeading(targetAvoidHeading)) {
        stopMotors();
        writeUltrasonicServo(90);
        classAvoidRetries = 0;
        addLog("🛑 Deadlock escape complete (180° Left turn). Mission PAUSED — "
               "click 'Start Mission' to resume.");
        strncpy(autoStatusStr, "PAUSED (DEADLOCK)", sizeof(autoStatusStr));
        classStep = CLASS_MISSION_DONE;
      }
      break;
    }
    break;
  }
}

// ======================== HTML & WEB DASHBOARD UI ============================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Auto Waste Bin — Control Center V2</title>
  <style>
    :root {
      --bg: #0d1117; --card: rgba(22,27,34,0.85); --border: rgba(255,255,255,0.1);
      --accent: #2f81f7; --accent2: #58a6ff; --green: #238636; --red: #da3633;
      --orange: #d29922; --purple: #8957e5; --text: #c9d1d9; --muted: #8b949e;
      --term: #090d13;
    }
    *{box-sizing:border-box;margin:0;padding:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
    body{background:var(--bg);color:var(--text);padding:15px;min-height:100vh;display:flex;flex-direction:column;align-items:center}
    .container{max-width:920px;width:100%;display:flex;flex-direction:column;gap:18px}
    header{background:var(--card);border:1px solid var(--border);padding:18px;border-radius:14px;text-align:center;backdrop-filter:blur(10px)}
    header h1{font-size:1.55rem;color:#fff;margin-bottom:4px}
    header p{color:var(--muted);font-size:0.85rem}
    .tabs{display:flex;gap:8px;background:rgba(0,0,0,0.3);padding:6px;border-radius:10px;border:1px solid var(--border)}
    .tab-btn{flex:1;padding:11px;border:none;border-radius:8px;background:transparent;color:var(--muted);font-size:0.9rem;font-weight:600;cursor:pointer;transition:all .2s}
    .tab-btn.active{background:var(--accent);color:#fff;box-shadow:0 4px 12px rgba(47,129,247,.35)}
    .tab-btn.active-cam{background:var(--purple);color:#fff;box-shadow:0 4px 12px rgba(137,87,229,.35)}
    .card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:18px;backdrop-filter:blur(10px)}
    .card h2{font-size:1.05rem;color:#fff;margin-bottom:14px;display:flex;align-items:center;gap:8px;border-bottom:1px solid var(--border);padding-bottom:8px}
    .dpad-container{display:flex;flex-direction:column;align-items:center;gap:9px;margin:14px 0}
    .dpad-row{display:flex;gap:9px}
    .btn{background:#21262d;border:1px solid #30363d;color:#fff;padding:13px 20px;border-radius:10px;font-size:.95rem;font-weight:bold;cursor:pointer;transition:all .15s;user-select:none;touch-action:manipulation}
    .btn:active,.btn.pressed{background:var(--accent);transform:scale(.96)}
    .btn-stop{background:var(--red);border-color:#f85149}
    .btn-stop:active{background:#b62324}
    .control-group{display:flex;flex-direction:column;gap:11px}
    .slider-card{display:flex;flex-direction:column;gap:5px;background:rgba(0,0,0,.2);padding:11px;border-radius:8px;border:1px solid rgba(255,255,255,.05)}
    .slider-header{display:flex;justify-content:space-between;font-size:.88rem;font-weight:500}
    input[type=range]{width:100%;accent-color:var(--accent);cursor:pointer}
    .preset-btns{display:flex;gap:7px;margin-top:4px}
    .btn-sm{padding:5px 11px;font-size:.78rem;border-radius:6px;background:#30363d;border:none;color:#fff;cursor:pointer}
    .btn-sm:hover{background:var(--accent)}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:11px}
    .stat-box{background:rgba(0,0,0,.3);border:1px solid var(--border);padding:11px;border-radius:10px;text-align:center}
    .stat-val{font-size:1.25rem;font-weight:bold;color:#fff;margin-top:4px}
    .stat-label{font-size:.75rem;color:var(--muted);text-transform:uppercase;letter-spacing:.05em}
    .terminal{background:var(--term);border:1px solid #30363d;border-radius:10px;padding:14px;height:250px;overflow-y:auto;font-family:"Courier New",monospace;font-size:.82rem;color:#7ee787;display:flex;flex-direction:column;gap:3px}
    .log-entry{border-bottom:1px solid rgba(255,255,255,.03);padding-bottom:2px}
    .badge{display:inline-block;padding:3px 9px;border-radius:12px;font-size:.78rem;font-weight:bold}
    .badge-manual{background:rgba(47,129,247,.2);color:var(--accent);border:1px solid var(--accent)}
    .badge-auto{background:rgba(210,153,34,.2);color:var(--orange);border:1px solid var(--orange)}
    .badge-cam{background:rgba(137,87,229,.2);color:var(--purple);border:1px solid var(--purple)}
    .two-col{display:grid;grid-template-columns:1fr 1fr;gap:14px}
    .cam-status-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:11px;margin-bottom:14px}
    @media(max-width:680px){.two-col{grid-template-columns:1fr}.cam-status-grid{grid-template-columns:1fr 1fr}}
  </style>
</head>
<body>
<div class="container">
  <header>
    <h1>🗑️ Auto Waste Bin — Control Center V2</h1>
    <p>ESP32 SoftAP Dashboard &nbsp;|&nbsp; Real-Time Telemetry &nbsp;|&nbsp; Manual · Autonomous · CAM Guided</p>
  </header>

  <!-- MPU6050 Disconnect Alert Banner -->
  <div id="mpu-disconnect-banner" style="display:none;background:rgba(218,54,51,0.22);border:1px solid #da3633;color:#ff7b72;padding:12px 16px;border-radius:10px;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:10px">
    <div>
      <strong style="font-size:0.95rem">🚨 MPU6050 Gyroscope DISCONNECTED!</strong>
      <div style="font-size:.82rem;color:#c9d1d9;margin-top:2px">All motor operations and autonomous modes are HALTED for safety. Check I2C wiring (SDA=21, SCL=22).</div>
    </div>
    <button class="btn-sm" style="background:#238636;color:#fff;padding:7px 16px;font-size:.85rem" onclick="recoverMPU()">🔄 Re-connect MPU</button>
  </div>

  <!-- Mode Tabs -->
  <div class="tabs">
    <button class="tab-btn active" id="tab-manual-btn" onclick="switchMode('manual')">🎮 Manual Control</button>
    <button class="tab-btn" id="tab-auto-btn"   onclick="switchMode('auto')">🏫 3-Classroom Patrol</button>
    <button class="tab-btn" id="tab-cam-btn"    onclick="switchMode('cam')">📷 CAM Guided Auto</button>
  </div>

  <!-- Telemetry Bar -->
  <div class="grid">
    <div class="stat-box"><div class="stat-label">System Mode</div>
      <div class="stat-val" id="tele-mode"><span class="badge badge-manual">MANUAL</span></div></div>
    <div class="stat-box"><div class="stat-label">ESP32-CAM Link</div>
      <div class="stat-val" id="tele-cam-link"><span class="badge" style="background:rgba(218,54,51,0.25);color:#da3633;border:1px solid #da3633">🔴 NO ACK</span></div></div>
    <div class="stat-box"><div class="stat-label">Obstacle Distance</div>
      <div class="stat-val" id="tele-dist">-- cm</div></div>
    <div class="stat-box"><div class="stat-label">MPU6050 &amp; Yaw</div>
      <div class="stat-val" id="tele-yaw">0.0°</div></div>
    <div class="stat-box"><div class="stat-label">Motor State</div>
      <div class="stat-val" id="tele-motor">STOPPED</div></div>
  </div>

  <!-- TAB 1: MANUAL CONTROL -->
  <div id="manual-view">
    <div class="two-col">
      <div class="card">
        <h2>🎮 Driving Controls</h2>
        <div class="dpad-container">
          <button class="btn" onclick="drv('forward')">▲ FORWARD</button>
          <div class="dpad-row">
            <button class="btn" onclick="drv('left')">↺ CIRCLE LEFT</button>
            <button class="btn btn-stop" onclick="drv('stop')">🛑 STOP</button>
            <button class="btn" onclick="drv('right')">CIRCLE RIGHT ↻</button>
          </div>
          <button class="btn" onclick="drv('backward')">▼ BACKWARD</button>
        </div>
      </div>

      <div class="card">
        <h2>📡 Sensor Pan Servo</h2>
        <div class="control-group">
          <div class="slider-card">
            <div class="slider-header"><span>Ultrasonic Pan</span><span id="us-servo-val">90°</span></div>
            <input type="range" id="us-servo-slider" min="0" max="180" value="90" oninput="setUsServo(this.value)">
            <div class="preset-btns">
              <button class="btn-sm" onclick="setUsServo(0)">0° Right</button>
              <button class="btn-sm" onclick="setUsServo(90)">90° Centre</button>
              <button class="btn-sm" onclick="setUsServo(180)">180° Left</button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- SYSTEM SETTINGS & SPEED TUNING -->
    <div class="card" style="margin-top:14px">
      <h2>⚙️ System Settings &amp; Speed Tuning</h2>
      <p style="font-size:.82rem;color:var(--muted);margin-bottom:12px">
        Type the exact tuning values you want. Parameters are sent to the microcontroller only when you click <b>Save &amp; Apply</b> or press <b>Enter</b> in any box.
      </p>
      <div class="grid" style="grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px">
        <div class="slider-card" style="text-align:left">
          <div class="slider-header"><span>Drive Speed PWM</span><span id="badge-drive-eff" style="font-size:.78rem;color:#34d399"></span></div>
          <input type="number" id="txt-drive" class="cfg-input" placeholder="e.g. 110" min="50" max="255" value="110" style="width:100%;padding:7px 10px;background:#090d16;border:1px solid var(--border);border-radius:6px;color:#fff;font-family:monospace;font-size:.9rem;margin-top:6px">
        </div>
        <div class="slider-card" style="text-align:left">
          <div class="slider-header"><span style="color:#60a5fa">◀ Turn Left Speed</span><span id="badge-turn-left-eff" style="font-size:.78rem;color:#34d399"></span></div>
          <input type="number" id="txt-turn-left" class="cfg-input" placeholder="e.g. 180" min="50" max="255" value="180" style="width:100%;padding:7px 10px;background:#090d16;border:1px solid var(--border);border-radius:6px;color:#fff;font-family:monospace;font-size:.9rem;margin-top:6px">
        </div>
        <div class="slider-card" style="text-align:left">
          <div class="slider-header"><span style="color:#f472b6">Turn Right Speed ▶</span><span id="badge-turn-right-eff" style="font-size:.78rem;color:#34d399"></span></div>
          <input type="number" id="txt-turn-right" class="cfg-input" placeholder="e.g. 210" min="50" max="255" value="210" style="width:100%;padding:7px 10px;background:#090d16;border:1px solid var(--border);border-radius:6px;color:#fff;font-family:monospace;font-size:.9rem;margin-top:6px">
        </div>
        <div class="slider-card" style="text-align:left">
          <div class="slider-header"><span>Obstacle Threshold (cm)</span></div>
          <input type="number" id="txt-thresh" class="cfg-input" placeholder="e.g. 50" min="10" max="200" value="50" style="width:100%;padding:7px 10px;background:#090d16;border:1px solid var(--border);border-radius:6px;color:#fff;font-family:monospace;font-size:.9rem;margin-top:6px">
        </div>
        <div class="slider-card" style="text-align:left">
          <div class="slider-header"><span>CAM Steer Gain</span></div>
          <input type="number" id="txt-gain" class="cfg-input" placeholder="e.g. 2" min="1" max="10" value="2" style="width:100%;padding:7px 10px;background:#090d16;border:1px solid var(--border);border-radius:6px;color:#fff;font-family:monospace;font-size:.9rem;margin-top:6px">
        </div>
        <div class="slider-card" style="text-align:left">
          <div class="slider-header"><span style="color:#a78bfa">Heading Tolerance (± deg)</span></div>
          <input type="number" id="txt-tol" class="cfg-input" placeholder="e.g. 12.0" min="1" max="45" step="0.5" value="12.0" style="width:100%;padding:7px 10px;background:#090d16;border:1px solid var(--border);border-radius:6px;color:#fff;font-family:monospace;font-size:.9rem;margin-top:6px">
        </div>
        <div class="slider-card" style="text-align:left">
          <div class="slider-header"><span style="color:#fbbf24">⚡ Stall Check Interval (ms)</span></div>
          <input type="number" id="txt-stall-int" class="cfg-input" placeholder="e.g. 500" min="100" max="5000" step="50" value="500" style="width:100%;padding:7px 10px;background:#090d16;border:1px solid var(--border);border-radius:6px;color:#fff;font-family:monospace;font-size:.9rem;margin-top:6px">
        </div>
      </div>
      <div style="margin-top:12px;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px">
        <span id="cfg-settings-status" style="font-size:.84rem;color:#34d399"></span>
        <div style="display:flex;gap:8px">
          <button class="btn-sm" style="background:#21262d" onclick="resetYaw()">🔄 Reset Yaw to 0°</button>
          <button class="btn-sm" style="background:#238636;color:#fff;padding:8px 18px;font-size:.88rem" onclick="saveSettings()">💾 Save &amp; Apply Settings</button>
        </div>
      </div>
    </div>

    <!-- NETWORK & LAPTOP SERVER CONFIGURATION -->
    <div class="card" style="margin-top:14px">
      <h2>🌐 Laptop Vision Server &amp; Wi-Fi Sync</h2>
      <p style="font-size:.82rem;color:var(--muted);margin-bottom:12px">
        Update your laptop server IP without re-flashing. Credentials are saved in flash NVS and instantly synced to the ESP32-CAM via UDP.
      </p>
      <div class="grid" style="grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px">
        <div class="slider-card" style="text-align:left">
          <div class="slider-header"><span style="color:#38bdf8">💻 Laptop IP (Port 5000)</span></div>
          <input type="text" id="cfg-laptop-ip" placeholder="e.g. 10.218.193.152" style="width:100%;padding:7px 10px;background:#090d16;border:1px solid var(--border);border-radius:6px;color:#fff;font-family:monospace;font-size:.9rem;margin-top:6px">
        </div>
        <div class="slider-card" style="text-align:left">
          <div class="slider-header"><span style="color:#34d399">📶 Router Wi-Fi SSID</span></div>
          <input type="text" id="cfg-wifi-ssid" placeholder="e.g. Traffic_ESP" style="width:100%;padding:7px 10px;background:#090d16;border:1px solid var(--border);border-radius:6px;color:#fff;font-family:monospace;font-size:.9rem;margin-top:6px">
        </div>
        <div class="slider-card" style="text-align:left">
          <div class="slider-header"><span style="color:#f59e0b">🔑 Wi-Fi Password</span></div>
          <input type="password" id="cfg-wifi-pass" placeholder="Leave blank to keep" style="width:100%;padding:7px 10px;background:#090d16;border:1px solid var(--border);border-radius:6px;color:#fff;font-family:monospace;font-size:.9rem;margin-top:6px">
        </div>
      </div>
      <div style="margin-top:12px;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px">
        <span id="cfg-sync-status" style="font-size:.84rem;color:#34d399"></span>
        <button class="btn-sm" style="background:#238636;color:#fff;padding:8px 18px;font-size:.88rem" onclick="saveNetworkConfig()">💾 Save &amp; Sync to Camera</button>
      </div>
    </div>
  </div>

  <!-- TAB 2: 3-CLASSROOM PATROL AUTONOMOUS -->
  <div id="auto-view" style="display:none">
    <div class="card">
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:11px;flex-wrap:wrap;gap:8px">
        <h2>🏫 3-Classroom Autonomous Patrol</h2>
        <div style="display:flex;gap:8px">
          <button id="btn-start-auto" class="btn-sm" style="background:#238636;color:#fff;padding:8px 16px;font-size:.88rem" onclick="startClassroomMission()">▶ Start Mission</button>
          <button id="btn-stop-auto" class="btn-sm" style="background:var(--red);color:#fff;padding:8px 16px;font-size:.88rem" onclick="stopClassroomMission()">🛑 Stop / Pause</button>
          <button class="btn-sm" style="background:#21262d;color:#8b949e" onclick="switchMode('manual')">Exit to Manual</button>
        </div>
      </div>
      <div class="grid" style="grid-template-columns:repeat(auto-fit,minmax(140px,1fr));margin-bottom:14px">
        <div class="stat-box"><div class="stat-label">Mission Stage</div>
          <div class="stat-val" id="auto-status-val" style="font-size:1.0rem">IDLE</div></div>
        <div class="stat-box"><div class="stat-label">Target Room</div>
          <div class="stat-val" id="auto-room-val">1 of 3</div></div>
      </div>
      <div class="terminal" id="auto-terminal"><div class="log-entry">Waiting for autonomous patrol telemetry...</div></div>
    </div>
  </div>

  <!-- TAB 3: CAM GUIDED AUTONOMOUS -->
  <div id="cam-view" style="display:none">
    <div class="card">
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:11px;flex-wrap:wrap;gap:8px">
        <h2 style="margin-bottom:0;border:none;padding:0">📷 CAM Guided — Vision State</h2>
        <div style="display:flex;gap:8px;align-items:center">
          <span id="cam-paused-badge" style="display:none;padding:4px 10px;border-radius:12px;font-size:.78rem;font-weight:bold;background:rgba(218,54,51,.2);color:#da3633;border:1px solid #da3633">⏸ PAUSED</span>
          <button id="btn-cam-pause"  class="btn-sm" style="background:#d29922;color:#000" onclick="camPauseToggle(true)">⏸ Pause</button>
          <button id="btn-cam-resume" class="btn-sm" style="display:none;background:#238636" onclick="camPauseToggle(false)">▶ Resume</button>
          <button class="btn-sm" style="background:var(--red)" onclick="switchMode('manual')">🛑 Exit CAM Mode</button>
        </div>
      </div>
      <div class="cam-status-grid" style="grid-template-columns:repeat(auto-fit,minmax(130px,1fr))">
        <div class="stat-box"><div class="stat-label">Robot Sub-State</div>
          <div class="stat-val" id="cam-sub-val" style="font-size:1.05rem">IDLE</div></div>
        <div class="stat-box"><div class="stat-label">Robot Facing</div>
          <div class="stat-val" id="cam-facing-val">FRONT</div></div>
        <div class="stat-box"><div class="stat-label">Camera Pan</div>
          <div class="stat-val" id="cam-pan-val" style="font-size:1rem">CENTER (90°)</div></div>
        <div class="stat-box"><div class="stat-label">Scan Cycle</div>
          <div class="stat-val" id="cam-cycle-val">1 of 2</div></div>
        <div class="stat-box"><div class="stat-label">Target Angle</div>
          <div class="stat-val" id="cam-angle-val">0°</div></div>
      </div>

      <div class="terminal" id="cam-terminal" style="margin-top:14px"><div class="log-entry">Waiting for CAM telemetry...</div></div>
    </div>
  </div>
</div>

<script>
  let uiMode = 'manual';
  let camIsPaused = false;
  let lastLogsCount = -1;

  function switchMode(mode) {
    uiMode = mode;
    if (mode !== 'cam') { camIsPaused = false; }
    fetch('/api/cmd?mode=' + mode).then(r=>r.json()).then(updateUI);
    document.getElementById('manual-view').style.display = (mode==='manual') ? 'block' : 'none';
    document.getElementById('auto-view').style.display   = (mode==='auto')   ? 'block' : 'none';
    document.getElementById('cam-view').style.display    = (mode==='cam')    ? 'block' : 'none';
    document.getElementById('tab-manual-btn').className = 'tab-btn'+(mode==='manual'?' active':'');
    document.getElementById('tab-auto-btn').className   = 'tab-btn'+(mode==='auto'  ?' active':'');
    document.getElementById('tab-cam-btn').className    = 'tab-btn'+(mode==='cam'   ?' active-cam':'');
  }
  document.getElementById('manual-view').style.display = 'block';

  function startClassroomMission() {
    fetch('/api/cmd?start_auto=1').then(r=>r.json()).then(updateUI);
  }

  function stopClassroomMission() {
    fetch('/api/cmd?stop_auto=1').then(r=>r.json()).then(updateUI);
  }

  function recoverMPU() {
    fetch('/api/cmd?recover_mpu=1').then(r=>r.json()).then(updateUI);
  }

  function camPauseToggle(pause) {
    camIsPaused = pause;
    fetch('/api/cmd?' + (pause ? 'cam_pause=1' : 'cam_resume=1'));
    document.getElementById('btn-cam-pause').style.display  = pause ? 'none'  : 'inline-block';
    document.getElementById('btn-cam-resume').style.display = pause ? 'inline-block' : 'none';
    document.getElementById('cam-paused-badge').style.display = pause ? 'inline-block' : 'none';
  }

  function drv(dir) { if(uiMode!=='manual') return; fetch('/api/cmd?dir='+dir); }

  function setUsServo(val) {
    document.getElementById('us-servo-val').innerText = val+'°';
    document.getElementById('us-servo-slider').value  = val;
    fetch('/api/cmd?us_servo='+val);
  }

  function saveSettings() {
    let d = parseInt(document.getElementById('txt-drive').value, 10);
    let tl = parseInt(document.getElementById('txt-turn-left').value, 10);
    let tr = parseInt(document.getElementById('txt-turn-right').value, 10);
    let th = parseFloat(document.getElementById('txt-thresh').value);
    let g = parseInt(document.getElementById('txt-gain').value, 10);
    let tol = parseFloat(document.getElementById('txt-tol').value);
    let st = parseInt(document.getElementById('txt-stall-int').value, 10);

    // Fallbacks and safe bounds
    if (isNaN(d) || d < 40) d = 110;
    if (isNaN(tl) || tl < 50) tl = 180;
    if (isNaN(tr) || tr < 50) tr = 210;
    if (isNaN(th) || th < 10) th = 50.0;
    if (isNaN(g) || g < 1) g = 2;
    if (isNaN(tol) || tol < 1.0) tol = 1.0;
    if (isNaN(st) || st < 100) st = 500;

    let statusEl = document.getElementById('cfg-settings-status');
    if (statusEl) statusEl.innerText = 'Applying...';

    let url = '/api/settings?drive_speed=' + d +
              '&turn_left_speed=' + tl +
              '&turn_right_speed=' + tr +
              '&threshold=' + th +
              '&gain=' + g +
              '&tolerance=' + tol +
              '&stall_interval=' + st;

    fetch(url).then(r => r.json()).then(res => {
      if (statusEl) {
        statusEl.innerText = '✅ Settings applied to robot!';
        setTimeout(() => { statusEl.innerText = ''; }, 3500);
      }
    }).catch(err => {
      if (statusEl) statusEl.innerText = '❌ Error saving settings';
    });
  }

  function saveNetworkConfig() {
    let ip = document.getElementById('cfg-laptop-ip').value.trim();
    let ssid = document.getElementById('cfg-wifi-ssid').value.trim();
    let pass = document.getElementById('cfg-wifi-pass').value;
    let statusEl = document.getElementById('cfg-sync-status');
    if (statusEl) statusEl.innerText = 'Syncing to ESP32-CAM...';
    let url = '/api/settings?laptop_ip=' + encodeURIComponent(ip) + '&wifi_ssid=' + encodeURIComponent(ssid);
    if (pass.length > 0) url += '&wifi_pass=' + encodeURIComponent(pass);
    fetch(url).then(r=>r.json()).then(d=>{
      if (statusEl) {
        statusEl.innerText = '✅ Saved & Synced to ESP32-CAM via UDP!';
        setTimeout(()=>{ statusEl.innerText = ''; }, 4000);
      }
    }).catch(()=>{ if (statusEl) statusEl.innerText = '❌ Error syncing config'; });
  }

  function resetYaw() { fetch('/api/cmd?reset_yaw=1'); }

  document.addEventListener('keydown', e => {
    if (['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName) || e.target.isContentEditable) {
      if (e.key === 'Enter') {
        if (e.target.classList.contains('cfg-input')) {
          e.preventDefault();
          saveSettings();
        } else if (['cfg-laptop-ip', 'cfg-wifi-ssid', 'cfg-wifi-pass'].includes(e.target.id)) {
          e.preventDefault();
          saveNetworkConfig();
        }
      }
      return;
    }
    if (uiMode !== 'manual' || e.repeat) return;
    if (e.key === 'w' || e.key === 'W') drv('forward');
    if (e.key === 's' || e.key === 'S') drv('backward');
    if (e.key === 'a' || e.key === 'A') drv('left');
    if (e.key === 'd' || e.key === 'D') drv('right');
    if (e.key === ' ' || e.key === 'Escape') drv('stop');
  });

  let slidersInitialized = false;

  function poll() {
    fetch('/api/telemetry').then(r=>r.json()).then(updateUI).catch(()=>{});
  }

  function updateUI(d) {
    if(!d) return;

    if(!slidersInitialized && d.drive_speed !== undefined) {
      if (document.getElementById('txt-drive')) document.getElementById('txt-drive').value = d.drive_speed;
      if (document.getElementById('txt-turn-left')) document.getElementById('txt-turn-left').value = (d.turn_left_speed !== undefined) ? d.turn_left_speed : 180;
      if (document.getElementById('txt-turn-right')) document.getElementById('txt-turn-right').value = (d.turn_right_speed !== undefined) ? d.turn_right_speed : 210;
      if (document.getElementById('txt-thresh')) document.getElementById('txt-thresh').value = d.threshold;
      if (document.getElementById('txt-gain')) document.getElementById('txt-gain').value = d.gain;
      if (document.getElementById('txt-tol')) document.getElementById('txt-tol').value = d.tolerance ? d.tolerance.toFixed(1) : '12.0';
      if (document.getElementById('txt-stall-int')) document.getElementById('txt-stall-int').value = (d.stall_interval !== undefined) ? d.stall_interval : 500;
      if (document.getElementById('cfg-laptop-ip') && d.laptop_ip !== undefined) document.getElementById('cfg-laptop-ip').value = d.laptop_ip;
      if (document.getElementById('cfg-wifi-ssid') && d.wifi_ssid !== undefined) document.getElementById('cfg-wifi-ssid').value = d.wifi_ssid;
      slidersInitialized = true;
    }

    // Reflect dynamic stall compensation adjustments on web interface in real time
    if (d.stall_boost_drive !== undefined) {
      const el = document.getElementById('badge-drive-eff');
      if (el) el.innerText = d.stall_boost_drive > 0 ? `Eff: ${d.drive_speed + d.stall_boost_drive} (+${d.stall_boost_drive})` : '';
    }
    if (d.stall_boost_left !== undefined) {
      const el = document.getElementById('badge-turn-left-eff');
      if (el) el.innerText = d.stall_boost_left > 0 ? `Eff: ${d.turn_left_speed + d.stall_boost_left} (+${d.stall_boost_left})` : '';
    }
    if (d.stall_boost_right !== undefined) {
      const el = document.getElementById('badge-turn-right-eff');
      if (el) el.innerText = d.stall_boost_right > 0 ? `Eff: ${d.turn_right_speed + d.stall_boost_right} (+${d.stall_boost_right})` : '';
    }

    let mb='<span class="badge badge-manual">MANUAL</span>';
    if(d.mode==='AUTONOMOUS') mb='<span class="badge badge-auto">AUTONOMOUS</span>';
    if(d.mode==='CAM_GUIDED') mb='<span class="badge badge-cam">CAM GUIDED</span>';
    document.getElementById('tele-mode').innerHTML = mb;
    document.getElementById('tele-dist').innerText  = d.ultrasonic>=0 ? d.ultrasonic.toFixed(1)+' cm' : 'Out of Range';
    document.getElementById('tele-motor').innerText = d.motor_state;

    if (d.mpu_connected !== undefined) {
      let mpuOk = d.mpu_connected;
      let banner = document.getElementById('mpu-disconnect-banner');
      if (banner) banner.style.display = mpuOk ? 'none' : 'flex';
      let yawEl = document.getElementById('tele-yaw');
      if (yawEl) {
        if (mpuOk) {
          yawEl.innerHTML = '<span style="color:#34d399;font-size:0.82rem">🟢 </span>' + (d.mpu_yaw>=0?'+':'') + d.mpu_yaw.toFixed(1) + '°';
        } else {
          yawEl.innerHTML = '<span class="badge" style="background:rgba(218,54,51,0.3);color:#ff7b72;border:1px solid #da3633">🔴 DISCONNECTED</span>';
        }
      }
    } else {
      document.getElementById('tele-yaw').innerText   = (d.mpu_yaw>=0?'+':'')+d.mpu_yaw.toFixed(1)+'°';
    }

    if(d.cam_online !== undefined) {
      const el = document.getElementById('tele-cam-link');
      if (d.cam_online) {
        el.innerHTML = '<span class="badge" style="background:rgba(35,134,54,0.25);color:#2ea043;border:1px solid #2ea043">🟢 ONLINE (' + d.cam_ago.toFixed(1) + 's)</span>';
      } else {
        el.innerHTML = '<span class="badge" style="background:rgba(218,54,51,0.25);color:#da3633;border:1px solid #da3633">🔴 NO ACK</span>';
      }
    }

    if(d.cam_paused !== undefined) {
      let p = d.cam_paused;
      document.getElementById('btn-cam-pause').style.display   = p ? 'none'  : 'inline-block';
      document.getElementById('btn-cam-resume').style.display  = p ? 'inline-block' : 'none';
      document.getElementById('cam-paused-badge').style.display = p ? 'inline-block' : 'none';
    }

    if(d.auto_sub !== undefined) {
      document.getElementById('auto-status-val').innerText = d.auto_sub || 'IDLE';
    }
    if(d.classroom !== undefined) {
      document.getElementById('auto-room-val').innerText = d.classroom + ' of 3';
    }

    if(d.cam_sub!==undefined) {
      document.getElementById('cam-sub-val').innerText    = d.cam_sub||'IDLE';
      document.getElementById('cam-facing-val').innerText = d.robot_facing||'FRONT';
      document.getElementById('cam-pan-val').innerText    = d.cam_pan||'CENTER (90°)';
      document.getElementById('cam-cycle-val').innerText  = d.scan_cycle||'1 of 2';
      document.getElementById('cam-angle-val').innerText  = (d.cam_angle>=0?'+':'')+d.cam_angle+'°';
    }

    if(d.total_logs !== undefined && d.total_logs !== lastLogsCount && d.logs && d.logs.length) {
      lastLogsCount = d.total_logs;
      let html = d.logs.map(l=>'<div class="log-entry">'+l+'</div>').join('');
      document.getElementById('auto-terminal').innerHTML = html;
      document.getElementById('auto-terminal').scrollTop = 9999;
      document.getElementById('cam-terminal').innerHTML  = html;
      document.getElementById('cam-terminal').scrollTop  = 9999;
    }
  }

  setInterval(poll, 1000);
</script>
</body>
</html>
)rawliteral";

// ======================== WEB HANDLERS =======================================
void handleRoot() { server.send(200, "text/html", INDEX_HTML); }

void handleTelemetry() {
  const char *modeStr = "MANUAL";
  if (currentMode == MODE_AUTONOMOUS)
    modeStr = "AUTONOMOUS";
  if (currentMode == MODE_CAM_GUIDED)
    modeStr = "CAM_GUIDED";

  bool camOnline = (lastCamRxMs > 0 && (millis() - lastCamRxMs < 4000));
  float camAgoSec =
      (lastCamRxMs > 0) ? ((millis() - lastCamRxMs) / 1000.0f) : 999.0f;

  const char *orientStr = "AHEAD (90°)";
  const char *cycleStr =
      (cgSub == CG_SCAN_CHECK || cgSub == CG_SCAN_TURN_90) ? "90° SCAN" : "--";
  const char *panStr = "CENTER (90°)";

  static char jsonBuf[2048];
  int pos = 0;
  pos += snprintf(
      jsonBuf + pos, sizeof(jsonBuf) - pos,
      "{"
      "\"mode\":\"%s\","
      "\"cam_online\":%s,"
      "\"cam_ago\":%.1f,"
      "\"mpu_connected\":%s,"
      "\"auto_sub\":\"%s\","
      "\"classroom\":%d,"
      "\"ultrasonic\":%.1f,"
      "\"mpu_yaw\":%.2f,"
      "\"motor_state\":\"%s\","
      "\"drive_speed\":%d,"
      "\"turn_left_speed\":%d,"
      "\"turn_right_speed\":%d,"
      "\"stall_interval\":%lu,"
      "\"stall_boost_left\":%d,"
      "\"stall_boost_right\":%d,"
      "\"stall_boost_drive\":%d,"
      "\"threshold\":%.1f,"
      "\"gain\":%d,"
      "\"tolerance\":%.1f,"
      "\"laptop_ip\":\"%s\","
      "\"wifi_ssid\":\"%s\","
      "\"us_servo\":%d,"
      "\"cam_state\":%d,"
      "\"cam_angle\":%d,"
      "\"cam_sub\":\"%s\","
      "\"robot_facing\":\"%s\","
      "\"scan_cycle\":\"%s\","
      "\"cam_pan\":\"%s\","
      "\"cam_paused\":%s,"
      "\"total_logs\":%d,"
      "\"logs\":[",
      modeStr, camOnline ? "true" : "false", camAgoSec,
      mpuConnected ? "true" : "false", autoStatusStr, currentClassroom,
      latestDistanceCm, currentYaw, currentMotorState, defaultDriveSpeed,
      turnLeftSpeed, turnRightSpeed, stallCheckIntervalMs, stallBoostLeft,
      stallBoostRight, stallBoostDrive, obstacleThresholdCm, camSteerGain,
      headingToleranceDeg, currentLaptopIp, currentWifiSsid, ultrasonicAngle,
      lastCamPkt.state, lastCamPkt.angleOffset, camStateStr, orientStr,
      cycleStr, panStr, camPaused ? "true" : "false", totalLogsAdded);

  int count = (totalLogsAdded < MAX_LOGS) ? totalLogsAdded : MAX_LOGS;
  int startIdx = (totalLogsAdded < MAX_LOGS) ? 0 : logHead;
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % MAX_LOGS;
    pos += snprintf(jsonBuf + pos, sizeof(jsonBuf) - pos, "\"%s\"%s",
                    decisionLogs[idx], (i < count - 1) ? "," : "");
    if (pos >= (int)sizeof(jsonBuf) - 10)
      break;
  }

  snprintf(jsonBuf + pos, sizeof(jsonBuf) - pos, "]}");
  server.send(200, "application/json", jsonBuf);
}

void handleCommand() {
  lastManualCmdMs = millis();

  if (server.hasArg("recover_mpu")) {
    resetI2C();
    if (mpuConnected) {
      calibrateMPU();
      addLog("✅ [WEB] MPU6050 recovered and calibrated successfully!");
    } else {
      addLog("❌ [WEB] MPU6050 recovery failed — check hardware wiring "
             "(SDA=21, SCL=22).");
    }
    broadcastMode();
  }

  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    if (!mpuConnected && m != "manual") {
      stopMotors();
      addLog("❌ [SAFETY REJECT] Cannot switch to autonomous/cam mode — "
             "MPU6050 is DISCONNECTED!");
      server.send(400, "application/json",
                  "{\"error\":\"MPU6050 is disconnected!\"}");
      return;
    }
    if (m == "auto") {
      currentMode = MODE_AUTONOMOUS;
      classStep = CLASS_INIT;
      classAvoidRetries = 0;
      camPaused = false;
      addLog("🔄 Mode → AUTONOMOUS (3-Classroom Patrol)");
    } else if (m == "cam") {
      currentMode = MODE_CAM_GUIDED;
      cgSub = CG_IDLE;
      cgAvoidRetries = 0;
      camPaused = false;
      strncpy(camStateStr, "IDLE", sizeof(camStateStr));
      addLog("🔄 Mode → CAM GUIDED (Vision Node)");
    } else {
      currentMode = MODE_MANUAL;
      camPaused = false;
      stopMotors();
      addLog("🔄 Mode → MANUAL");
    }
    broadcastMode();
  }

  if (server.hasArg("start_auto")) {
    if (!mpuConnected) {
      stopMotors();
      addLog("❌ [SAFETY REJECT] Cannot start 3-Classroom Mission — MPU6050 is "
             "DISCONNECTED!");
      server.send(400, "application/json",
                  "{\"error\":\"MPU6050 is disconnected!\"}");
      return;
    }
    currentMode = MODE_AUTONOMOUS;
    classStep = CLASS_INIT;
    classAvoidRetries = 0;
    camPaused = false;
    addLog("▶ [WEB] Starting 3-Classroom Patrol Mission...");
    broadcastMode();
  }

  if (server.hasArg("stop_auto")) {
    stopMotors();
    classStep = CLASS_MISSION_DONE;
    strncpy(autoStatusStr, "STOPPED / PAUSED", sizeof(autoStatusStr));
    addLog("⏸️ [WEB] 3-Classroom Patrol Mission stopped / paused.");
    broadcastMode();
  }

  if (server.hasArg("cam_pause") && currentMode == MODE_CAM_GUIDED) {
    camPaused = true;
    stopMotors();
    broadcastMode();
    addLog("⏸ CAM Guided PAUSED by user.");
  }
  if (server.hasArg("cam_resume") && currentMode == MODE_CAM_GUIDED) {
    camPaused = false;
    cgSub = CG_IDLE;
    broadcastMode();
    addLog("▶ CAM Guided RESUMED — restarting scan.");
  }

  if (currentMode == MODE_MANUAL && server.hasArg("dir")) {
    String dir = server.arg("dir");
    if (dir == "forward") {
      manualDriveState = MANUAL_DRIVE_FORWARD;
      addLog("▶ [MANUAL] Cruising FORWARD (Auto obstacle avoidance active)");
    } else if (dir == "backward") {
      manualDriveState = MANUAL_DRIVE_BACKWARD;
      addLog("▶ [MANUAL] Cruising BACKWARD");
    } else if (dir == "left") {
      manualDriveState = MANUAL_DRIVE_CIRCLE_LEFT;
      addLog("↺ [MANUAL] Circling LEFT (Auto obstacle avoidance active)");
    } else if (dir == "right") {
      manualDriveState = MANUAL_DRIVE_CIRCLE_RIGHT;
      addLog("↻ [MANUAL] Circling RIGHT (Auto obstacle avoidance active)");
    } else if (dir == "stop") {
      manualDriveState = MANUAL_IDLE;
      stopMotors();
      writeUltrasonicServo(90);
      addLog("🛑 [MANUAL] Motors STOPPED by user.");
    }
  }

  if (server.hasArg("us_servo")) {
    writeUltrasonicServo(server.arg("us_servo").toInt());
  }
  if (server.hasArg("reset_yaw")) {
    calibrateMPU();
    currentYaw = 0.0f;
    addLog("🔄 MPU re-calibrated and Yaw zeroed.");
  }
  handleTelemetry();
}

void loadSettings() {
  prefs.begin("robot_cfg", true); // Read-only
  defaultDriveSpeed = prefs.getInt("drive_spd", 110);
  turnLeftSpeed = prefs.getInt("turn_l_spd", prefs.getInt("turn_spd", 180));
  turnRightSpeed = prefs.getInt("turn_r_spd", prefs.getInt("turn_spd", 210));
  obstacleThresholdCm = prefs.getFloat("obs_thresh", 50.0f);
  camSteerGain = prefs.getInt("steer_gain", 2);
  headingToleranceDeg = prefs.getFloat("head_tol", 12.0f);
  stallCheckIntervalMs = (unsigned long)prefs.getInt("stall_int", 500);
  if (stallCheckIntervalMs < 100 || stallCheckIntervalMs > 5000)
    stallCheckIntervalMs = 500;
  if (headingToleranceDeg < 1.0f || headingToleranceDeg > 45.0f)
    headingToleranceDeg = 12.0f;
  if (prefs.isKey("laptop_ip"))
    prefs.getString("laptop_ip", currentLaptopIp, sizeof(currentLaptopIp));
  if (prefs.isKey("wifi_ssid"))
    prefs.getString("wifi_ssid", currentWifiSsid, sizeof(currentWifiSsid));
  if (prefs.isKey("wifi_pass"))
    prefs.getString("wifi_pass", currentWifiPass, sizeof(currentWifiPass));
  prefs.end();
  Serial.printf("📦 Loaded from EEPROM/NVS: Drive=%d, TurnL=%d, TurnR=%d, "
                "StallInt=%lums, Thresh=%.1fcm, Gain=%d, HeadTol=%.1f°, "
                "LaptopIP=%s, SSID='%s'\n",
                defaultDriveSpeed, turnLeftSpeed, turnRightSpeed,
                stallCheckIntervalMs, obstacleThresholdCm, camSteerGain,
                headingToleranceDeg, currentLaptopIp, currentWifiSsid);
}

void saveSettings() {
  prefs.begin("robot_cfg", false); // Read-write
  prefs.putInt("drive_spd", defaultDriveSpeed);
  prefs.putInt("turn_l_spd", turnLeftSpeed);
  prefs.putInt("turn_r_spd", turnRightSpeed);
  prefs.putFloat("obs_thresh", obstacleThresholdCm);
  prefs.putInt("steer_gain", camSteerGain);
  prefs.putFloat("head_tol", headingToleranceDeg);
  prefs.putInt("stall_int", (int)stallCheckIntervalMs);
  prefs.putString("laptop_ip", currentLaptopIp);
  prefs.putString("wifi_ssid", currentWifiSsid);
  prefs.putString("wifi_pass", currentWifiPass);
  prefs.end();
  addLog("💾 Settings & Network Config saved to EEPROM / NVS.");
}

void handleSettings() {
  bool netChanged = false;
  bool wifiChanged = false;

  if (server.hasArg("drive_speed")) {
    int v = server.arg("drive_speed").toInt();
    if (v >= 40 && v <= 255)
      defaultDriveSpeed = v;
  }
  if (server.hasArg("turn_left_speed")) {
    int v = server.arg("turn_left_speed").toInt();
    if (v >= 50 && v <= 255)
      turnLeftSpeed = v;
  }
  if (server.hasArg("turn_right_speed")) {
    int v = server.arg("turn_right_speed").toInt();
    if (v >= 50 && v <= 255)
      turnRightSpeed = v;
  }
  if (server.hasArg("turn_speed") && !server.hasArg("turn_left_speed") &&
      !server.hasArg("turn_right_speed")) {
    int v = server.arg("turn_speed").toInt();
    if (v >= 50 && v <= 255) {
      turnLeftSpeed = v;
      turnRightSpeed = v;
    }
  }
  if (server.hasArg("threshold")) {
    float v = server.arg("threshold").toFloat();
    if (v >= 10.0f && v <= 200.0f)
      obstacleThresholdCm = v;
  }
  if (server.hasArg("gain")) {
    int v = server.arg("gain").toInt();
    if (v >= 1 && v <= 10)
      camSteerGain = v;
  }
  if (server.hasArg("tolerance")) {
    float v = server.arg("tolerance").toFloat();
    if (v >= 1.0f && v <= 45.0f)
      headingToleranceDeg = v;
  }
  if (server.hasArg("stall_interval")) {
    int v = server.arg("stall_interval").toInt();
    if (v >= 100 && v <= 5000)
      stallCheckIntervalMs = (unsigned long)v;
  }

  if (server.hasArg("laptop_ip")) {
    String lip = server.arg("laptop_ip");
    lip.trim();
    if (lip.length() >= 7 &&
        strncmp(currentLaptopIp, lip.c_str(), sizeof(currentLaptopIp)) != 0) {
      strncpy(currentLaptopIp, lip.c_str(), sizeof(currentLaptopIp));
      netChanged = true;
    }
  }
  if (server.hasArg("wifi_ssid")) {
    String wssid = server.arg("wifi_ssid");
    wssid.trim();
    if (wssid.length() > 0 &&
        strncmp(currentWifiSsid, wssid.c_str(), sizeof(currentWifiSsid)) != 0) {
      strncpy(currentWifiSsid, wssid.c_str(), sizeof(currentWifiSsid));
      netChanged = true;
      wifiChanged = true;
    }
  }
  if (server.hasArg("wifi_pass")) {
    String wpass = server.arg("wifi_pass");
    wpass.trim();
    if (wpass.length() > 0 &&
        strncmp(currentWifiPass, wpass.c_str(), sizeof(currentWifiPass)) != 0) {
      strncpy(currentWifiPass, wpass.c_str(), sizeof(currentWifiPass));
      netChanged = true;
      wifiChanged = true;
    }
  }

  saveSettings(); // Persist changes immediately to flash EEPROM/NVS

  if (netChanged) {
    broadcastMode(1); // Broadcast full config sync to ESP32-CAM via UDP!
    char netLog[128];
    snprintf(netLog, sizeof(netLog),
             "📡 Config synced to Camera: Laptop IP=%s, Wi-Fi='%s'",
             currentLaptopIp, currentWifiSsid);
    addLog(netLog);
    if (wifiChanged) {
      Serial.printf("🔄 Connecting Main ESP32 to new Wi-Fi: '%s'...\n",
                    currentWifiSsid);
      WiFi.disconnect();
      WiFi.begin(currentWifiSsid, currentWifiPass);
    }
  } else {
    char logBuf[128];
    snprintf(logBuf, sizeof(logBuf),
             "⚙️ Settings updated: Drive=%d TurnL=%d TurnR=%d Thresh=%.0fcm "
             "Gain=%d Tol=%.1f°",
             defaultDriveSpeed, turnLeftSpeed, turnRightSpeed,
             obstacleThresholdCm, camSteerGain, headingToleranceDeg);
    addLog(logBuf);
  }

  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// ======================== SETUP ==============================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG,
                 0); // Disable brownout detector to prevent voltage dip reboots
  Serial.begin(115200);
  delay(1000);
  Serial.println(
      F("\n============================================================"));
  Serial.println(
      F("   AUTO WASTE BIN V2 — MAIN ESP32 NAVIGATION NODE          "));
  Serial.println(
      F("============================================================"));

  loadSettings(); // Load saved speeds, threshold, gain, and tolerance from
                  // EEPROM/NVS

  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  ultrasonicServo.setPeriodHertz(50);
  camServo.setPeriodHertz(50);

  // Attach servos and command initial center positions
  writeUltrasonicServo(90);
  writeCamServo(CAM_ANGLE_CENTER);

  int mPins[] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB};
  for (int p : mPins)
    pinMode(p, OUTPUT);
  stopMotors();
  Serial.println(F("✅ Motor driver pins configured"));

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.println(F("✅ Ultrasonic sensor ready"));

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(25);
  if (!mpu.begin()) {
    Serial.println(F("⚠️  MPU6050 not found — continuing without IMU."));
  } else {
    mpuConnected = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    applyMPUHardwareOffsets();
    gyroZ_offset = CALIBRATED_GYRO_Z_OFFSET;
    Serial.println(F("✅ MPU6050 connected with calibrated offsets"));
  }

  // Configure Wi-Fi in pure Station (STA) Mode on router to eliminate SoftAP
  // radio overhead:
  WiFi.mode(WIFI_STA);
  WiFi.begin(currentWifiSsid, currentWifiPass);
  Serial.printf("Connecting to Wi-Fi '%s'...", currentWifiSsid);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 25) {
    delay(400);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ Joined Wi-Fi '%s' — IP: http://%s (Channel: %d)\n",
                  currentWifiSsid, WiFi.localIP().toString().c_str(),
                  WiFi.channel());
  } else {
    Serial.println(F("\n⚠️ Wi-Fi router unreachable — activating emergency "
                     "SoftAP fallback."));
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("⚠️ Emergency SoftAP active: http://%s\n",
                  WiFi.softAPIP().toString().c_str());
  }

  // Start mDNS responder so dashboard is accessible via http://autowaste.local
  if (MDNS.begin("autowaste")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println(F("✅ mDNS responder started: http://autowaste.local"));
  } else {
    Serial.println(F("⚠️ Error starting mDNS responder!"));
  }

  server.on("/", handleRoot);
  server.on("/api/telemetry", handleTelemetry);
  server.on("/api/cmd", handleCommand);
  server.on("/api/settings", handleSettings);
  server.begin();
  Serial.println(F("✅ Web server running on port 80"));

  // Start UDP for peer-to-peer Wi-Fi router messaging with ESP32-CAM
  udp.begin(UDP_MAIN_RX_PORT);
  Serial.printf("✅ UDP listening on port %d (Target: %d)\n", UDP_MAIN_RX_PORT,
                UDP_CAM_RX_PORT);

  lastSampleUs = micros();
  lastManualCmdMs = millis();

  delay(500);
  broadcastMode();

  char readyMsg[128];
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(
        readyMsg, sizeof(readyMsg),
        "🚀 System ready! Web Dashboard: http://autowaste.local (or http://%s)",
        WiFi.localIP().toString().c_str());
  } else {
    snprintf(readyMsg, sizeof(readyMsg),
             "🚀 System ready! SoftAP: http://autowaste.local (or http://%s)",
             WiFi.softAPIP().toString().c_str());
  }
  addLog(readyMsg);
}

// ======================== MAIN LOOP ==========================================
unsigned long lastSensorMs = 0;

void loop() {
  updateMPU();
  serviceStallCompensation();
  serviceServos();
  serviceBrake();
  checkUdpPackets();
  server.handleClient();

  if (millis() - lastSensorMs >= 100) {
    lastSensorMs = millis();
    latestDistanceCm = readUltrasonic();
  }

  processManualMode();
  processClassroomAuto();
  processCamGuided();
}
