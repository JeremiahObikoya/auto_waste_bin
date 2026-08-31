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

#include <Arduino.h>
#include "secrets.h" // ← Credentials & UDP ports
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

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

// Camera servo pan angles for scan cycle: Front/Center (90°), Left (45°), Right (135°)
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

// ======================== GLOBAL SENSOR / MOTOR STATE ========================
float latestDistanceCm = -1.0f;
char currentMotorState[16] = "STOPPED";
int currentSpeedA = 0;
int currentSpeedB = 0;
int ultrasonicPos = 90;

// ======================== SYSTEM PARAMETERS & NETWORK CONFIG =================
float obstacleThresholdCm = 50.0f; // Safety stop distance (cm)
int defaultDriveSpeed = 110;
int turnLeftSpeed = 180;           // Calibrated minimum static friction breakaway for LEFT (180 PWM)
int turnRightSpeed = 210;          // Calibrated minimum static friction breakaway for RIGHT (210 PWM)
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

void addLog(const String &msg) {
  addLog(msg.c_str());
}

// ======================== MPU6050 CALIBRATED OFFSETS & I2C AUTO-RECOVERY =====
// Calibrated from MPU6050 Web Calibration Tool:
const float CALIBRATED_GYRO_Z_OFFSET = 0.000141f; // 0.008 deg/s zero-rate bias

const int16_t HW_ACCEL_OFFSET_X = -3556;
const int16_t HW_ACCEL_OFFSET_Y = 272;
const int16_t HW_ACCEL_OFFSET_Z = 1336;
const int16_t HW_GYRO_OFFSET_X  = 91;
const int16_t HW_GYRO_OFFSET_Y  = -17;
const int16_t HW_GYRO_OFFSET_Z  = 12;

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
  if (!mpuConnected) return;

  // Read existing Temperature Compensation (TC) bits from low byte registers (Bit 0)
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
    if (g.gyro.z < minGz) minGz = g.gyro.z;
    if (g.gyro.z > maxGz) maxGz = g.gyro.z;
    delay(3);
  }
  // Check if robot was moving during calibration (gyro spread > 0.08 rad/s ~ 4.5°/s)
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
  snprintf(offBuf, sizeof(offBuf), "✅ MPU6050 calibrated. Offset: %.5f rad/s (%.3f °/s)", gyroZ_offset, gyroZ_offset * (180.0f / PI));
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

void updateMPU() {
  if (!mpuConnected)
    return;
  unsigned long now = micros();
  float dt = (now - lastSampleUs) / 1000000.0f;
  lastSampleUs = now;

  // Guard against initial boot lag or loop delay spikes
  if (dt <= 0.0f || dt > 0.05f)
    dt = 0.01f;

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
}

// ======================== SERVO JITTER FIX (AUTO-DETACH) =====================
int ultrasonicAngle = 90;
int camAngleWritten = CAM_ANGLE_CENTER;
bool ultrasonicAttached = false;
bool camAttached = false;
unsigned long ultrasonicMoveMs = 0;
unsigned long camMoveMs = 0;

void writeUltrasonicServo(int angle) {
  angle = constrain(angle, 0, 180);
  if (angle != ultrasonicAngle || !ultrasonicAttached) {
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

void writeCamServo(int angle) {
  angle = constrain(angle, 0, 180);
  if (angle != camAngleWritten || !camAttached) {
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
// Automatically detects chassis stalls due to battery discharge or surface friction.
// - Turning: monitors Gyro Z angular rate & interval-based Yaw delta.
// - Driving: monitors Accelerometer motion surge/vibration.
// If robot is stalled over each check window, increases PWM by +12 step-by-step until motion is observed.
unsigned long stallCheckIntervalMs = 400; // Global tunable stall check interval (default 400ms)
int stallBoostLeft = 0;
int stallBoostRight = 0;
int stallBoostDrive = 0;
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
}

// Non-blocking active electrical brake: drives all IN pins HIGH simultaneously to
// short the motor back-EMF, applying maximum magnetic stopping torque.
// Asynchronously serviced and released to coast by serviceBrake() after BRAKE_PULSE_DURATION_MS.
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
  if (motorBrakingActive && (millis() - motorBrakeStartMs >= BRAKE_PULSE_DURATION_MS)) {
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
  if (!mpuConnected || mpuErrorCount > 0 || isnan(gyroEv.gyro.z) || isnan(accelEv.acceleration.x))
    return;

  // If stopped or braking, keep tracking reset
  if (strcmp(currentMotorState, "STOPPED") == 0 || strcmp(currentMotorState, "BRAKING") == 0) {
    if (strcmp(lastObservedMotorState, "STOPPED") != 0) {
      strncpy(lastObservedMotorState, currentMotorState, sizeof(lastObservedMotorState));
      motionStateStartMs = 0;
      lastStallBoostMs = 0;
      isStalled = false;
    }
    return;
  }

  unsigned long now = millis();

  // If motion command state changed (e.g. STOPPED -> LEFT or FORWARD -> RIGHT)
  if (strcmp(currentMotorState, lastObservedMotorState) != 0) {
    strncpy(lastObservedMotorState, currentMotorState, sizeof(lastObservedMotorState));
    motionStateStartMs = now;
    lastStallBoostMs = now;
    lastRecordedYaw = currentYaw;
    lastRecordedAccelX = accelEv.acceleration.x;
    lastRecordedAccelY = accelEv.acceleration.y;
    isStalled = false;
    return;
  }

  // Initial grace period for motor magnetic field to energize & attempt movement
  if (now - motionStateStartMs < stallCheckIntervalMs)
    return;

  // Check moving window progress every stallCheckIntervalMs
  if (now - lastStallBoostMs < stallCheckIntervalMs)
    return;

  // ── A. TURNING STALL DETECTION (LEFT / RIGHT) ─────────────────────────────
  if (strcmp(currentMotorState, "LEFT") == 0 || strcmp(currentMotorState, "RIGHT") == 0) {
    float deltaYawInterval = fabsf(currentYaw - lastRecordedYaw);
    float rotRate = fabsf(currentGyroRateZ);

    // If robot is stalled: very low angular velocity AND minimal yaw delta during this interval
    if (rotRate < 3.5f && deltaYawInterval < 0.8f) {
      isStalled = true;
      if (strcmp(currentMotorState, "LEFT") == 0) {
        stallBoostLeft = min(stallBoostLeft + 12, 255 - turnLeftSpeed);
        int newSpd = constrain(turnLeftSpeed + stallBoostLeft, turnLeftSpeed, 255);
        setMotorSpeeds(newSpd, newSpd);
        Serial.printf("⚡ [STALL BOOST -> LEFT] Yaw: %6.1f° stuck! Rate: %4.1f°/s, Delta: %4.1f° in %lums | Boost: +%d -> Applied PWM: %d\n",
                      currentYaw, rotRate, deltaYawInterval, stallCheckIntervalMs, stallBoostLeft, newSpd);
      } else {
        stallBoostRight = min(stallBoostRight + 12, 255 - turnRightSpeed);
        int newSpd = constrain(turnRightSpeed + stallBoostRight, turnRightSpeed, 255);
        setMotorSpeeds(newSpd, newSpd);
        Serial.printf("⚡ [STALL BOOST -> RIGHT] Yaw: %6.1f° stuck! Rate: %4.1f°/s, Delta: %4.1f° in %lums | Boost: +%d -> Applied PWM: %d\n",
                      currentYaw, rotRate, deltaYawInterval, stallCheckIntervalMs, stallBoostRight, newSpd);
      }
    } else if (rotRate >= 5.0f || deltaYawInterval >= 1.2f) {
      if (isStalled) {
        isStalled = false;
        Serial.printf("🚀 [MOTION RESUMED -> %s] Yaw: %6.1f° | Rate: %5.1f°/s | Delta: %4.1f° | Boost: +%d | Applied PWM: %d\n",
                      currentMotorState, currentYaw, rotRate, deltaYawInterval,
                      (strcmp(currentMotorState, "LEFT") == 0 ? stallBoostLeft : stallBoostRight),
                      currentSpeedA);
      }
    }

    lastStallBoostMs = now;
    lastRecordedYaw = currentYaw;
  }

  // ── B. LINEAR DRIVE STALL DETECTION (FORWARD / BACKWARD) ──────────────────
  else if (strcmp(currentMotorState, "FORWARD") == 0 || strcmp(currentMotorState, "BACKWARD") == 0) {
    float deltaAx = fabsf(accelEv.acceleration.x - lastRecordedAccelX);
    float deltaAy = fabsf(accelEv.acceleration.y - lastRecordedAccelY);
    float motionJerk = deltaAx + deltaAy;

    // If rolling chassis vibration or linear surge is absent (< 0.15 m/s^2):
    if (motionJerk < 0.15f) {
      stallBoostDrive = min(stallBoostDrive + 10, 255 - defaultDriveSpeed);
      int newSpd = constrain(defaultDriveSpeed + stallBoostDrive, defaultDriveSpeed, 255);
      setMotorSpeeds(newSpd, newSpd);
      Serial.printf("⚡ [STALL BOOST -> DRIVE] Forward/Back sag! Boost: +%d -> Applied PWM: %d\n",
                    stallBoostDrive, newSpd);
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

// ======================== UDP WI-FI ROUTER COMMUNICATION =====================
struct __attribute__((packed)) CamPacket {
  uint8_t state; // 0=SCANNING, 1=TRACKING, 2=INTERACTION, 3=RELEASE, 4=SCAN_STEP_DONE, 5=OBSTACLE_CHECK, 6=IS_TARGET, 7=IS_OBSTACLE
  int8_t angleOffset;
};

struct __attribute__((packed)) MainPacket {
  uint8_t pktType;    // 0=Mode/Heartbeat, 1=Config Sync, 5=Trigger Obstacle Check
  uint8_t systemMode; // 0=MANUAL, 1=AUTONOMOUS, 2=CAM_GUIDED, 3=PAUSED, 5=OBSTACLE_CHECK
  char laptopIp[32];
  char wifiSsid[33];
  char wifiPass[65];
};

CamPacket lastCamPkt = {0, 0};
bool newCamPkt = false;
unsigned long lastCamRxMs = 0; // Timestamp of last received packet from ESP32-CAM
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
  Serial.printf("[UDP TX] systemMode=%d, laptopIp=%s, pktType=%d\n", pkt.systemMode, pkt.laptopIp, pktType);
}

void checkUdpPackets() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    if (packetSize == sizeof(CamPacket)) {
      udp.read((uint8_t *)&lastCamPkt, sizeof(CamPacket));
      newCamPkt = true;
      lastCamRxMs = millis();
      Serial.printf("[UDP RX] CamPacket: state=%d, angleOffset=%d\n", lastCamPkt.state, lastCamPkt.angleOffset);
    } else {
      // Discard unexpected broadcast packets (e.g. router mDNS, PC discovery) to prevent LwIP buffer leaks
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
//      - When |targetAngle - currentYaw| <= tolerance, triggers non-blocking active electrical brake.
//      - Settle dwell timer requires error to remain <= tolerance continuously for SETTLE_WINDOW_MS (180ms)
//        AND the 80ms electrical brake pulse to complete before returning true.
//   2. Deceleration Zone (absErr < 30°):
//      - Dynamically scales down turn speed toward base breakaway speed.
//      - Dynamic stall boost (+stallBoost) is automatically added if static friction prevents reaching target.
//   3. Far Zone (absErr >= 30°):
//      - Starts at base breakaway speed and smoothly ramps up (+25 PWM / 1000ms) up to 255 PWM.
//   4. Automatic Breakaway Kick / Unwedge Routine:
//      - If robot is stuck at maximum power (>245 PWM / max boost) for >= 1200ms with zero rotation (<0.8°),
//        executes a brief 180ms swing-pivot or reverse kick to overcome static floor friction / caster bind.
bool rotateToHeading(float targetAngle, float tolerance = -1.0f) {
  // Use configured tolerance if none specified
  if (tolerance <= 0.0f)
    tolerance = headingToleranceDeg;

  // Enforce a safe floor (at least 1.0°) so a low NVS value never causes oscillation
  if (tolerance < 1.0f)
    tolerance = 1.0f;

  // ── No MPU fallback: timed blind turn ───────────────────────────────────────
  if (!mpuConnected) {
    static unsigned long noMpuTimer = 0;
    if (noMpuTimer == 0)
      noMpuTimer = millis();
    if (millis() - noMpuTimer >= 800) {
      noMpuTimer = 0;
      stopMotors();
      return true;
    }
    turnRight(turnRightSpeed);
    return false;
  }

  // ── Compute shortest-path error (-180° to +180°) ───────────────────────────
  float error = targetAngle - currentYaw;
  while (error >  180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
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

  const unsigned long SETTLE_WINDOW_MS = 180; // Required continuous in-tolerance dwell time

  // ── ACCEPTANCE & SETTLE WINDOW (absErr <= tolerance) ───────────────────────
  if (absErr <= tolerance) {
    unwedgeActive = false;
    maxPwmStallTimer = 0;
    unwedgeAttemptCount = 0;

    // First cycle entering the tolerance window: start dwell timer and trigger active brake
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
      Serial.printf("⏳ [IN TOLERANCE] Target: %6.1f° | Yaw: %6.1f° | Err: %5.1f° (Tol: ±%.1f°) | Dwell: %lu/%lums | PWM: (%d, %d) | Brake: %s\n",
                    targetAngle, currentYaw, error, tolerance,
                    (millis() - settleStartMs), SETTLE_WINDOW_MS,
                    currentSpeedA, currentSpeedB,
                    motorBrakingActive ? "ACTIVE" : "RELEASED");
    }

    // Verify dwell condition: must remain continuously in tolerance for SETTLE_WINDOW_MS
    // AND the non-blocking brake pulse must have fully released.
    if ((millis() - settleStartMs >= SETTLE_WINDOW_MS) && !motorBrakingActive) {
      stopMotors();
      Serial.printf("🎯 [TURN LOCKED] Target: %6.1f° | Final Yaw: %6.1f° | Final Err: %5.1f° | Settled in window -> ADVANCING!\n",
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

  int8_t currentTurnDir = (error > 0) ? 1 : -1; // +1 = Left (increases yaw), -1 = Right (decreases yaw)

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

  // Smooth linear far-zone ramp speed: base breakaway PWM + 25 PWM / 1000ms up to 255
  unsigned long elapsedMs = millis() - rampStartMs;
  int rampAdd = (int)((elapsedMs * 25) / 1000);
  int farSpd = constrain(baseSpd + rampAdd, baseSpd, 255);

  int spd = farSpd;

  // Deceleration zone: fixed 30° boundary based on physical chassis stopping distance.
  const float decelBoundary = 30.0f; // Degrees before target to begin slowing down

  if (absErr < decelBoundary) {
    float decelFactor = constrain((absErr - tolerance) / (decelBoundary - tolerance), 0.0f, 1.0f);
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
          // Swing-Pivot Kick: power the outer wheel forward (255 PWM), let inner wheel coast (0 PWM)
          // Rolling leverage breaks static friction and instantly flips the caster wheel!
          Serial.printf("⚡ [BREAKAWAY KICK #%d] Stalled at 255 PWM for 1.2s! Applying SWING-PIVOT pulse (outer wheel forward)...\n", unwedgeAttemptCount);
          if (currentTurnDir > 0) {
            // Turning Left: drive right wheel (Motor B) forward, left wheel coast
            digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
            digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, HIGH);
            setMotorSpeeds(0, 255);
          } else {
            // Turning Right: drive left wheel (Motor A) forward, right wheel coast
            digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH);
            digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
            setMotorSpeeds(255, 0);
          }
        } else {
          // Reverse-Nudge Kick: brief reverse (180 PWM) to un-stick any physical binding
          Serial.printf("⚡ [BREAKAWAY KICK #%d] Stalled at 255 PWM for 1.2s! Applying REVERSE-NUDGE pulse...\n", unwedgeAttemptCount);
          digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
          digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
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
    Serial.printf("🧭 [TURN %-5s] Target: %6.1f° | Yaw: %6.1f° | Err: %5.1f° (Tol: ±%.1f°) | Rate: %5.1f°/s | Base: %d | Ramp: +%d | StallBoost: +%d | ENA(L): %3d | ENB(R): %3d | Zone: %s\n",
                  (currentTurnDir > 0 ? "LEFT" : "RIGHT"),
                  targetAngle, currentYaw, error, tolerance,
                  currentGyroRateZ, baseSpd, (spd - baseSpd), activeBoost,
                  currentSpeedA, currentSpeedB,
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
  CG_IDLE,           // Initial entry / reset
  CG_SCANNING_CYCLE, // Stepping camera servo: Center (90°) → Left (45°) → Right (135°)
  CG_TURN_180_REAR,  // Robot rotating 180° with MPU6050 to face rear
  CG_TURN_180_FRONT, // Robot rotating 180° with MPU6050 back to front
  CG_IDLE_WAIT,      // 10 s idle rest period
  CG_TRACKING,       // Person detected — moving forward & curve-steering
  CG_CHECK_OBSTACLE, // Front obstacle detected while tracking — query Gemini (target vs blocker)
  CG_AVOID_OBSTACLE, // Obstacle in path confirmed blocker — scanning left/right and steering around
  CG_PRESENT_TURN,   // Turn 90° Left upon reaching target to present bin
  CG_WAIT_DROP,      // Wait 5 seconds for waste drop
  CG_RESTORE_TURN,   // Turn 90° Right back to approach orientation
  CG_BACKOFF_ROTATE  // 180° departure turn to face other side and resume scan
};

CamGuidedSub cgSub = CG_IDLE;
FacingOrientation robotOrientation = ORIENTATION_FRONT;
int currentCycle = 1; // 1 or 2
int currentStep = 0;  // 0=CENTER (90°), 1=LEFT (45°), 2=RIGHT (135°)
unsigned long stepStartMs = 0;
unsigned long idleStartMs = 0;
unsigned long cgReverseStart = 0;
unsigned long haltStartMs = 0;
unsigned long obstacleCheckTimer = 0;
int avoidStep = 0;
unsigned long avoidTimer = 0;
float leftAvoidDist = -1.0f;
float rightAvoidDist = -1.0f;
float cgRotateTarget = 0.0f;
float cgSavedApproachYaw = 0.0f;
char camStateStr[64] = "IDLE";
int8_t camAngleOffset = 0;

const char *getStepName(int step) {
  if (step == 0)
    return "CENTER (90°)";
  if (step == 1)
    return "LEFT (45°)";
  if (step == 2)
    return "RIGHT (135°)";
  return "--";
}

void processCamGuided() {
  if (currentMode != MODE_CAM_GUIDED)
    return;
  if (camPaused) {
    stopMotors();
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
      cgRotateTarget = cgSavedApproachYaw; // Turn 90° RIGHT back to original approach orientation
      addLog("⏰ 5s elapsed / drop confirmed! Turning 90° RIGHT back to approach heading...");
      cgSub = CG_RESTORE_TURN;
      strncpy(camStateStr, "TURNING 90° R", sizeof(camStateStr));
    }
    return;
  }

  // ── 90° Turn Right to Restore Original Heading ────────────────────────────
  if (cgSub == CG_RESTORE_TURN) {
    strncpy(camStateStr, "TURNING 90° R", sizeof(camStateStr));
    if (rotateToHeading(cgRotateTarget)) {
      stopMotors();
      cgRotateTarget = currentYaw + 180.0f; // Turn 180° to face other side
      addLog("🔄 Aligned! Starting 180° departure turn to face other side...");
      cgSub = CG_BACKOFF_ROTATE;
      strncpy(camStateStr, "ROTATING 180°", sizeof(camStateStr));
    }
    return;
  }

  // ── 180° Departure Turn to Face Other Side & Scan ─────────────────────────
  if (cgSub == CG_BACKOFF_ROTATE) {
    strncpy(camStateStr, "ROTATING 180°", sizeof(camStateStr));
    if (rotateToHeading(cgRotateTarget)) {
      stopMotors();
      addLog("✅ Departure turn complete — ready for next scan.");
      writeCamServo(CAM_ANGLE_CENTER);
      writeUltrasonicServo(90);
      cgSub = CG_IDLE;
      strncpy(camStateStr, "IDLE", sizeof(camStateStr));
    }
    return;
  }

  // ── 180° MPU Turn to Face REAR ────────────────────────────────────────────
  if (cgSub == CG_TURN_180_REAR) {
    strncpy(camStateStr, "ROTATING 180° TO REAR", sizeof(camStateStr));
    if (rotateToHeading(cgRotateTarget)) {
      stopMotors();
      robotOrientation = ORIENTATION_REAR;
      currentCycle = 1;
      currentStep = 0;
      writeCamServo(CAM_ANGLE_CENTER);
      stepStartMs = millis();
      addLog("✅ 180° turn complete — robot is now facing REAR.");
      addLog("📡 [REAR] Cycle 1/2 — checking CENTER (90°)...");
      cgSub = CG_SCANNING_CYCLE;
      strncpy(camStateStr, "SCANNING REAR (1/2 CENTER)", sizeof(camStateStr));
    }
    return;
  }

  // ── 180° MPU Turn to Face FRONT ───────────────────────────────────────────
  if (cgSub == CG_TURN_180_FRONT) {
    strncpy(camStateStr, "ROTATING 180° TO FRONT", sizeof(camStateStr));
    if (rotateToHeading(cgRotateTarget)) {
      stopMotors();
      robotOrientation = ORIENTATION_FRONT;
      idleStartMs = millis();
      addLog("✅ 180° turn complete — robot is now facing FRONT.");
      addLog("💤 No targets found in 360°. Entering 10-second Idle Mode...");
      cgSub = CG_IDLE_WAIT;
      strncpy(camStateStr, "IDLE — WAITING (10s)", sizeof(camStateStr));
    }
    return;
  }

  // ── Target Detected during Scanning Cycle ─────────────────────────────────
  if (cgSub == CG_SCANNING_CYCLE && isNewPacket && pkt.state == 1) {
    String orientStr = (robotOrientation == ORIENTATION_FRONT) ? "FRONT" : "REAR";
    addLog("🎯 Target detected facing " + orientStr + "! Centering camera and approaching target...");
    writeCamServo(CAM_ANGLE_CENTER);
    cgSub = CG_TRACKING;
    snprintf(camStateStr, sizeof(camStateStr), "TRACKING (%s)", orientStr.c_str());
    curveSteer(pkt.angleOffset);
    return;
  }

  // ── State Machine Processing ──────────────────────────────────────────────
  switch (cgSub) {

  case CG_IDLE:
    stopMotors();
    robotOrientation = ORIENTATION_FRONT;
    currentCycle = 1;
    currentStep = 0;
    writeCamServo(CAM_ANGLE_CENTER);
    stepStartMs = now;
    addLog("📡 [FRONT] Autonomous scan started — Cycle 1/2: checking CENTER (90°)...");
    cgSub = CG_SCANNING_CYCLE;
    strncpy(camStateStr, "SCANNING FRONT (1/2 CENTER)", sizeof(camStateStr));
    break;

  case CG_SCANNING_CYCLE: {
    stopMotors();
    String orient = (robotOrientation == ORIENTATION_FRONT) ? "FRONT" : "REAR";
    snprintf(camStateStr, sizeof(camStateStr), "SCANNING %s (%d/2 %s)",
             orient.c_str(), currentCycle, getStepName(currentStep));

    // ONLY advance to next angle when ESP32-CAM confirms a completed SCAN_STEP_DONE (state 4)
    if (isNewPacket && pkt.state == 4) {
      currentStep++;
      Serial.printf("📷 [UDP SCAN STEP] Received SCAN_STEP_DONE from ESP32-CAM -> Advancing to step %d (%s)\n",
                    currentStep, getStepName(currentStep));

      if (currentStep == 1) {
        writeCamServo(CAM_ANGLE_LEFT);
        stepStartMs = millis();
        addLog("🔍 [" + orient + "] Cycle " + String(currentCycle) + "/2 — checking LEFT (45°)...");
      } else if (currentStep == 2) {
        writeCamServo(CAM_ANGLE_RIGHT);
        stepStartMs = millis();
        addLog("🔍 [" + orient + "] Cycle " + String(currentCycle) + "/2 — checking RIGHT (135°)...");
      } else {
        currentStep = 0;
        writeCamServo(CAM_ANGLE_CENTER);
        currentCycle++;

        if (currentCycle <= 2) {
          stepStartMs = millis();
          addLog("🔍 [" + orient + "] Starting Scan Cycle 2/2 — checking CENTER (90°)...");
        } else {
          if (robotOrientation == ORIENTATION_FRONT) {
            addLog("❌ 2 cycles completed facing FRONT — no one detected.");
            addLog("🔄 Rotating robot 180° with MPU6050 to check REAR...");
            cgRotateTarget = currentYaw + 180.0f;
            cgSub = CG_TURN_180_REAR;
            strncpy(camStateStr, "ROTATING 180° TO REAR", sizeof(camStateStr));
          } else {
            addLog("❌ 2 cycles completed facing REAR — no one detected.");
            addLog("🔄 Rotating robot 180° with MPU6050 back to FRONT...");
            cgRotateTarget = currentYaw + 180.0f;
            cgSub = CG_TURN_180_FRONT;
            strncpy(camStateStr, "ROTATING 180° TO FRONT", sizeof(camStateStr));
          }
        }
      }
    }
    break;
  }

  case CG_IDLE_WAIT: {
    stopMotors();
    static unsigned long lastIdleLogMs = 0;
    if (now - lastIdleLogMs >= 3000) {
      lastIdleLogMs = now;
      long remaining = (IDLE_DURATION_MS - (now - idleStartMs)) / 1000;
      if (remaining > 0)
        addLog("⏳ Idle — resuming scan in " + String(remaining) + " s...");
    }
    if (now - idleStartMs >= IDLE_DURATION_MS) {
      addLog("⏰ Idle complete. Restarting Autonomous Scan routine...");
      cgSub = CG_IDLE;
    }
    break;
  }

  case CG_TRACKING:
    strncpy(camStateStr, "TRACKING", sizeof(camStateStr));
    if (pkt.state == 1) {
      // Obstacle detected in path while approaching person -> verify if obstacle is target or blocker
      if (latestDistanceCm > 0 && latestDistanceCm < obstacleThresholdCm) {
        stopMotors();
        writeCamServo(CAM_ANGLE_CENTER);
        writeUltrasonicServo(90);
        cgSub = CG_CHECK_OBSTACLE;
        obstacleCheckTimer = now;
        strncpy(camStateStr, "VERIFYING OBSTACLE", sizeof(camStateStr));
        addLog("🚧 Obstacle at " + String(latestDistanceCm, 1) + " cm! Querying Gemini to verify if it is target or blocker...");
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

    // Handle response from ESP32-CAM (states 6 = IS_TARGET, 7 = IS_OBSTACLE, 2 = INTERACTION, 3 = RELEASE)
    if (isNewPacket) {
      if (pkt.state == 6 || pkt.state == 2 || pkt.state == 3) {
        stopMotors();
        writeCamServo(CAM_ANGLE_CENTER);
        writeUltrasonicServo(90);
        cgSavedApproachYaw = currentYaw;
        cgRotateTarget = currentYaw + 90.0f; // Turn 90° Left (+90°)
        addLog("🎯 Gemini confirmed obstacle IS the target! Turning 90° LEFT to present bin...");
        cgSub = CG_PRESENT_TURN;
        strncpy(camStateStr, "PRESENTING BIN (90° L)", sizeof(camStateStr));
      } else if (pkt.state == 7 || pkt.state == 0) {
        addLog("🚧 Gemini confirmed obstacle is an UNRELATED BLOCKER. Starting avoidance swerve...");
        avoidStep = 0;
        avoidTimer = now;
        writeUltrasonicServo(120); // Look left 120°
        cgSub = CG_AVOID_OBSTACLE;
        strncpy(camStateStr, "AVOIDING OBSTACLE", sizeof(camStateStr));
      }
    }

    // Timeout fallback after 10 seconds if no Gemini answer arrives
    if (now - obstacleCheckTimer >= 10000) {
      addLog("⏱️ Gemini verification timed out — treating as obstacle to avoid.");
      avoidStep = 0;
      avoidTimer = now;
      writeUltrasonicServo(120);
      cgSub = CG_AVOID_OBSTACLE;
    }
    break;
  }

  case CG_AVOID_OBSTACLE: {
    strncpy(camStateStr, "AVOIDING OBSTACLE", sizeof(camStateStr));
    static int cgAvoidRetries = 0;
    if (pkt.state == 0) {
      addLog("⚠️ Target lost during avoidance — restarting scan.");
      stopMotors();
      writeUltrasonicServo(90);
      cgAvoidRetries = 0;
      cgSub = CG_IDLE;
      return;
    }
    switch (avoidStep) {
    case 0: // Scan Left 120° (300ms settle)
      writeUltrasonicServo(120);
      if (now - avoidTimer >= 300) {
        leftAvoidDist = readUltrasonic();
        writeUltrasonicServo(60); // Look right 60°
        avoidTimer = now;
        avoidStep = 1;
      }
      break;
    case 1: // Scan Right 60° (300ms settle)
      if (now - avoidTimer >= 300) {
        rightAvoidDist = readUltrasonic();
        writeUltrasonicServo(90); // Re-center sonar
        avoidTimer = now;
        avoidStep = 2;
      }
      break;
    case 2: // Decide path (wait 250ms for center servo to settle)
      if (now - avoidTimer >= 250) {
        float currentDist = readUltrasonic();
        if (currentDist <= 0 || currentDist > obstacleThresholdCm) {
          addLog("✅ Path clear — resuming camera tracking.");
          cgAvoidRetries = 0;
          cgSub = CG_TRACKING;
        } else {
          cgAvoidRetries++;
          if (cgAvoidRetries >= 3) {
            addLog("⚠️ Path remained blocked after 3 attempts — reversing and re-planning...");
            moveBackward(defaultDriveSpeed);
            delay(1000);
            stopMotors();
            cgAvoidRetries = 0;
            cgSub = CG_TRACKING;
          } else {
            float l = (leftAvoidDist <= 0) ? 999.0f : leftAvoidDist;
            float r = (rightAvoidDist <= 0) ? 999.0f : rightAvoidDist;
            if (l > r && l > obstacleThresholdCm) {
              addLog("💡 Swerving LEFT around obstacle");
              turnLeft(turnLeftSpeed);
              avoidTimer = now;
              avoidStep = 3;
            } else if (r >= l && r > obstacleThresholdCm) {
              addLog("💡 Swerving RIGHT around obstacle");
              turnRight(turnRightSpeed);
              avoidTimer = now;
              avoidStep = 3;
            } else {
              addLog("⚠️ Both sides narrow — backing up...");
              moveBackward(defaultDriveSpeed);
              avoidTimer = now;
              avoidStep = 3;
            }
          }
        }
      }
      break;
    case 3: // Complete swerve step (wait 350ms)
      if (now - avoidTimer >= 350) {
        stopMotors();
        cgSub = CG_TRACKING;
      }
      break;
    }
    break;
  }

  default:
    break;
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

ClassStep returnAfterAvoidStep = CLASS_CORRIDOR_DRIVE;
float returnAfterAvoidHeading = 0.0f;
int avoidSubStep = 0;
int classAvoidRetries = 0;
unsigned long avoidSubTimer = 0;
float avoidLeftDist = -1.0f;
float avoidRightDist = -1.0f;
char autoStatusStr[32] = "IDLE";

void processClassroomAuto() {
  if (currentMode != MODE_AUTONOMOUS)
    return;

  unsigned long now = millis();

  // Telemetry stream for 3-Classroom Autonomous Mode
  static unsigned long lastClassLogMs = 0;
  if (now - lastClassLogMs >= 200) {
    lastClassLogMs = now;
    Serial.printf("🏫 [CLASS AUTO] State: %-26s | Yaw: %6.1f° | Target: %6.1f° | CorridorBase: %6.1f° | Dist: %5.1f cm | Motor: %-8s (%d, %d)\n",
                  autoStatusStr, currentYaw, targetHeading, corridorHeading, latestDistanceCm, currentMotorState, currentSpeedA, currentSpeedB);
  }

  switch (classStep) {

  case CLASS_INIT:
    currentClassroom = 1;
    corridorHeading = currentYaw; // Baseline corridor heading
    targetHeading = corridorHeading;
    targetDurationMs = 2000;      // 2 seconds forward
    classStepTimer = now;
    classAvoidRetries = 0;
    writeUltrasonicServo(90);
    writeCamServo(CAM_ANGLE_CENTER);
    addLog("🏫 Starting 3-Classroom Patrol Mission (Classroom 1/3)...");
    addLog("🚗 [Room 1/3] Moving forward along corridor (2.0s)...");
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM 1/3 — CORRIDOR (2s)");
    classStep = CLASS_CORRIDOR_DRIVE;
    break;

  // 1. Move Forward along corridor (2s)
  case CLASS_CORRIDOR_DRIVE:
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — CORRIDOR (2s)", currentClassroom);
    moveForward(defaultDriveSpeed);

    // Obstacle Check
    if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
      stopMotors();
      remainingMoveMs = (now - classStepTimer < targetDurationMs) ? (targetDurationMs - (now - classStepTimer)) : 100;
      returnAfterAvoidStep = CLASS_CORRIDOR_DRIVE;
      returnAfterAvoidHeading = corridorHeading;
      avoidSubStep = 0;
      avoidSubTimer = now;
      classAvoidRetries = 0;
      addLog("🚨 Obstacle in corridor — checking clearance...");
      classStep = CLASS_OBSTACLE_AVOID;
      break;
    }

    if (now - classStepTimer >= targetDurationMs) {
      stopMotors();
      targetHeading = corridorHeading + 90.0f; // Turn LEFT 90° into room (+90° CCW)
      addLog("🚪 At classroom entrance. Turning LEFT 90° into room...");
      snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — TURN LEFT 90°", currentClassroom);
      classStep = CLASS_TURN_ROOM;
    }
    break;

  // 2. Turn Left 90° into Classroom
  case CLASS_TURN_ROOM:
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — TURN LEFT 90°", currentClassroom);
    if (rotateToHeading(targetHeading)) {
      addLog("➡️ Heading locked! Moving forward into room (2.0s)...");
      targetDurationMs = 2000;
      classStepTimer = millis();
      snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — ENTERING (2s)", currentClassroom);
      classStep = CLASS_ENTER_ROOM;
    }
    break;

  // 3. Move Forward into Classroom (2s)
  case CLASS_ENTER_ROOM:
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — ENTERING (2s)", currentClassroom);
    moveForward(defaultDriveSpeed);

    // Obstacle Check
    if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
      stopMotors();
      remainingMoveMs = (now - classStepTimer < targetDurationMs) ? (targetDurationMs - (now - classStepTimer)) : 100;
      returnAfterAvoidStep = CLASS_ENTER_ROOM;
      returnAfterAvoidHeading = targetHeading;
      avoidSubStep = 0;
      avoidSubTimer = now;
      classAvoidRetries = 0;
      addLog("🚨 Obstacle inside classroom!");
      classStep = CLASS_OBSTACLE_AVOID;
      break;
    }

    if (now - classStepTimer >= targetDurationMs) {
      stopMotors();
      targetHeading = corridorHeading + 180.0f; // Turn 90° LEFT inside classroom (+90° room entry + 90° = +180°)
      addLog("🚪 Inside classroom. Turning 90° LEFT to present waste bin...");
      snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — PRESENT BIN (90° L)", currentClassroom);
      classStep = CLASS_PRESENT_TURN;
    }
    break;

  // 4. Turn 90° Left to Present Bin in Classroom
  case CLASS_PRESENT_TURN:
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — PRESENT BIN (90° L)", currentClassroom);
    if (rotateToHeading(targetHeading)) {
      stopMotors();
      classStepTimer = millis();
      addLog("🛑 Bin presented! Waiting 5.0s for waste disposal...");
      snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — WAITING (5s)", currentClassroom);
      classStep = CLASS_WAIT_DROP;
    }
    break;

  // 5. Wait 5s in Classroom for Waste Drop
  case CLASS_WAIT_DROP:
    stopMotors();
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — WAITING (5s)", currentClassroom);
    if (now - classStepTimer >= 5000) {
      targetHeading = corridorHeading + 90.0f; // Turn 90° RIGHT back to room entry/exit heading
      addLog("⏰ 5s elapsed! Turning 90° RIGHT to face room exit...");
      snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — RESTORE HEADING", currentClassroom);
      classStep = CLASS_RESTORE_TURN;
    }
    break;

  // 6. Turn 90° Right back to Exit Heading
  case CLASS_RESTORE_TURN:
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — RESTORE HEADING", currentClassroom);
    if (rotateToHeading(targetHeading)) {
      stopMotors();
      targetDurationMs = 2000;
      classStepTimer = millis();
      addLog("⬅️ Collection complete! Reversing out of room (2.0s)...");
      snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — EXITING (2s)", currentClassroom);
      classStep = CLASS_EXIT_ROOM;
    }
    break;

  // 7. Move Backward out of Classroom (2s)
  case CLASS_EXIT_ROOM:
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — EXITING (2s)", currentClassroom);
    moveBackward(defaultDriveSpeed);

    if (now - classStepTimer >= targetDurationMs) {
      stopMotors();
      targetHeading = corridorHeading; // Turn back right 90° to corridor baseline
      addLog("🔄 Reached corridor. Turning RIGHT 90° to face path...");
      snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — TURN RIGHT 90°", currentClassroom);
      classStep = CLASS_TURN_CORRIDOR;
    }
    break;

  // 8. Turn Right 90° back to Corridor Heading
  case CLASS_TURN_CORRIDOR:
    snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — TURN RIGHT 90°", currentClassroom);
    if (rotateToHeading(targetHeading)) {
      addLog("✅ Classroom visit completed!");
      currentClassroom++;

      if (currentClassroom <= 3) {
        targetDurationMs = 2000; // Next classroom forward 2s
        classStepTimer = millis();
        addLog("🏫 Moving forward to next classroom (2.0s)...");
        snprintf(autoStatusStr, sizeof(autoStatusStr), "ROOM %d/3 — CORRIDOR (2s)", currentClassroom);
        classStep = CLASS_CORRIDOR_DRIVE;
      } else {
        targetHeading = corridorHeading + 180.0f; // Turn 180° for return
        addLog("🏁 All 3 classrooms visited! Turning 180° for Return Home Routine...");
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
      classStepTimer = millis();
      addLog("🚗 Returning home along corridor (Moving forward 6.0s)...");
      strncpy(autoStatusStr, "RETURNING HOME (6s)", sizeof(autoStatusStr));
      classStep = CLASS_RETURN_DRIVE;
    }
    break;

  // 8. Return Drive (6s)
  case CLASS_RETURN_DRIVE:
    strncpy(autoStatusStr, "RETURNING HOME (6s)", sizeof(autoStatusStr));
    moveForward(defaultDriveSpeed);

    // Obstacle Check
    if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
      stopMotors();
      remainingMoveMs = (now - classStepTimer < targetDurationMs) ? (targetDurationMs - (now - classStepTimer)) : 100;
      returnAfterAvoidStep = CLASS_RETURN_DRIVE;
      returnAfterAvoidHeading = targetHeading;
      avoidSubStep = 0;
      avoidSubTimer = now;
      classAvoidRetries = 0;
      addLog("🚨 Obstacle on return path!");
      classStep = CLASS_OBSTACLE_AVOID;
      break;
    }

    if (now - classStepTimer >= targetDurationMs) {
      stopMotors();
      targetHeading = corridorHeading; // Turn 180° back to original home orientation
      addLog("🏠 Reached home station. Aligning to original orientation...");
      strncpy(autoStatusStr, "ALIGNING HOME", sizeof(autoStatusStr));
      classStep = CLASS_RETURN_ALIGN;
    }
    break;

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

    // Sub-step 0: Scan Left 120°
    case 0:
      writeUltrasonicServo(120);
      if (now - avoidSubTimer >= 300) {
        avoidLeftDist = readUltrasonic();
        writeUltrasonicServo(60); // Scan Right 60°
        avoidSubTimer = now;
        avoidSubStep = 1;
      }
      break;

    // Sub-step 1: Scan Right 60°
    case 1:
      if (now - avoidSubTimer >= 300) {
        avoidRightDist = readUltrasonic();
        writeUltrasonicServo(90); // Re-center sonar
        avoidSubTimer = now;
        avoidSubStep = 2;
      }
      break;

    // Sub-step 2: Decision & Swerve
    case 2:
      if (now - avoidSubTimer >= 250) {
        float currentDist = readUltrasonic();
        if (currentDist <= 0 || currentDist > obstacleThresholdCm) {
          addLog("✅ Path is now clear — resuming patrol mission.");
          classStepTimer = millis() - (targetDurationMs - remainingMoveMs);
          classStep = returnAfterAvoidStep;
        } else {
          classAvoidRetries++;
          if (classAvoidRetries >= 3) {
            addLog("⚠️ Obstacle blocking path after 3 attempts — backing up...");
            moveBackward(defaultDriveSpeed);
            delay(1000);
            stopMotors();
            classAvoidRetries = 0;
            avoidSubStep = 0;
            avoidSubTimer = millis();
          } else {
            float l = (avoidLeftDist <= 0) ? 999.0f : avoidLeftDist;
            float r = (avoidRightDist <= 0) ? 999.0f : avoidRightDist;

            if (l > r && l > obstacleThresholdCm) {
              addLog("💡 Swerving LEFT around obstacle");
              turnLeft(turnLeftSpeed);
              avoidSubTimer = now;
              avoidSubStep = 3;
            } else if (r >= l && r > obstacleThresholdCm) {
              addLog("💡 Swerving RIGHT around obstacle");
              turnRight(turnRightSpeed);
              avoidSubTimer = now;
              avoidSubStep = 3;
            } else {
              addLog("⚠️ Both sides narrow — backing up...");
              moveBackward(defaultDriveSpeed);
              avoidSubTimer = now;
              avoidSubStep = 3;
            }
          }
        }
      }
      break;

    // Sub-step 3: Complete Swerve & Re-orient
    case 3:
      if (now - avoidSubTimer >= 350) {
        stopMotors();
        avoidSubStep = 4;
      }
      break;

    // Sub-step 4: Rotate back to mission heading
    case 4:
      if (rotateToHeading(returnAfterAvoidHeading)) {
        addLog("✅ Re-aligned to mission path. Resuming drive...");
        classStepTimer = millis() - (targetDurationMs - remainingMoveMs);
        classStep = returnAfterAvoidStep;
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
    <div class="stat-box"><div class="stat-label">MPU Yaw</div>
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
          <button class="btn" onmousedown="drv('forward')" onmouseup="drv('stop')" ontouchstart="drv('forward')" ontouchend="drv('stop')">▲ FORWARD</button>
          <div class="dpad-row">
            <button class="btn" onmousedown="drv('left')" onmouseup="drv('stop')" ontouchstart="drv('left')" ontouchend="drv('stop')">◀ LEFT</button>
            <button class="btn btn-stop" onclick="drv('stop')">🛑 STOP</button>
            <button class="btn" onmousedown="drv('right')" onmouseup="drv('stop')" ontouchstart="drv('right')" ontouchend="drv('stop')">RIGHT ▶</button>
          </div>
          <button class="btn" onmousedown="drv('backward')" onmouseup="drv('stop')" ontouchstart="drv('backward')" ontouchend="drv('stop')">▼ BACKWARD</button>
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
        <button class="btn-sm" style="background:var(--red)" onclick="switchMode('manual')">🛑 Stop Mission</button>
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
    if (e.key === ' ') drv('stop');
  });

  document.addEventListener('keyup', e => {
    if (['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName) || e.target.isContentEditable) return;
    if (uiMode !== 'manual') return;
    if (['w', 'W', 's', 'S', 'a', 'A', 'd', 'D'].includes(e.key)) drv('stop');
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
    document.getElementById('tele-yaw').innerText   = (d.mpu_yaw>=0?'+':'')+d.mpu_yaw.toFixed(1)+'°';
    document.getElementById('tele-motor').innerText = d.motor_state;

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
  float camAgoSec = (lastCamRxMs > 0) ? ((millis() - lastCamRxMs) / 1000.0f) : 999.0f;

  const char *orientStr = (robotOrientation == ORIENTATION_FRONT) ? "FRONT" : "REAR";
  char cycleStr[16] = "--";
  if (cgSub == CG_SCANNING_CYCLE) {
    snprintf(cycleStr, sizeof(cycleStr), "%d of 2", currentCycle);
  }
  const char *panStr = (cgSub == CG_SCANNING_CYCLE) ? getStepName(currentStep) : "CENTER (90°)";

  static char jsonBuf[2048];
  int pos = 0;
  pos += snprintf(jsonBuf + pos, sizeof(jsonBuf) - pos,
    "{"
    "\"mode\":\"%s\","
    "\"cam_online\":%s,"
    "\"cam_ago\":%.1f,"
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
    modeStr,
    camOnline ? "true" : "false",
    camAgoSec,
    autoStatusStr,
    currentClassroom,
    latestDistanceCm,
    currentYaw,
    currentMotorState,
    defaultDriveSpeed,
    turnLeftSpeed,
    turnRightSpeed,
    stallCheckIntervalMs,
    stallBoostLeft,
    stallBoostRight,
    stallBoostDrive,
    obstacleThresholdCm,
    camSteerGain,
    headingToleranceDeg,
    currentLaptopIp,
    currentWifiSsid,
    ultrasonicAngle,
    lastCamPkt.state,
    lastCamPkt.angleOffset,
    camStateStr,
    orientStr,
    cycleStr,
    panStr,
    camPaused ? "true" : "false",
    totalLogsAdded
  );

  int count = (totalLogsAdded < MAX_LOGS) ? totalLogsAdded : MAX_LOGS;
  int startIdx = (totalLogsAdded < MAX_LOGS) ? 0 : logHead;
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % MAX_LOGS;
    pos += snprintf(jsonBuf + pos, sizeof(jsonBuf) - pos, "\"%s\"%s", decisionLogs[idx], (i < count - 1) ? "," : "");
    if (pos >= (int)sizeof(jsonBuf) - 10) break;
  }

  snprintf(jsonBuf + pos, sizeof(jsonBuf) - pos, "]}");
  server.send(200, "application/json", jsonBuf);
}

void handleCommand() {
  lastManualCmdMs = millis();

  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    if (m == "auto") {
      currentMode = MODE_AUTONOMOUS;
      classStep = CLASS_INIT;
      camPaused = false;
      addLog("🔄 Mode → AUTONOMOUS (3-Classroom Patrol)");
    } else if (m == "cam") {
      currentMode = MODE_CAM_GUIDED;
      cgSub = CG_IDLE;
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
      if (latestDistanceCm > 0 && latestDistanceCm < obstacleThresholdCm) {
        stopMotors();
        addLog("🚨 Obstacle in front — cannot advance!");
      } else {
        moveForward();
      }
    } else if (dir == "backward") {
      moveBackward();
    } else if (dir == "left") {
      turnLeft();
    } else if (dir == "right") {
      turnRight();
    } else if (dir == "stop") {
      stopMotors();
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
  defaultDriveSpeed   = prefs.getInt("drive_spd", 110);
  turnLeftSpeed       = prefs.getInt("turn_l_spd", prefs.getInt("turn_spd", 180));
  turnRightSpeed      = prefs.getInt("turn_r_spd", prefs.getInt("turn_spd", 210));
  obstacleThresholdCm = prefs.getFloat("obs_thresh", 50.0f);
  camSteerGain        = prefs.getInt("steer_gain", 2);
  headingToleranceDeg = prefs.getFloat("head_tol", 12.0f);
  stallCheckIntervalMs = (unsigned long)prefs.getInt("stall_int", 500);
  if (stallCheckIntervalMs < 100 || stallCheckIntervalMs > 5000)
    stallCheckIntervalMs = 500;
  if (headingToleranceDeg < 1.0f || headingToleranceDeg > 45.0f)
    headingToleranceDeg = 12.0f;
  if (prefs.isKey("laptop_ip")) prefs.getString("laptop_ip", currentLaptopIp, sizeof(currentLaptopIp));
  if (prefs.isKey("wifi_ssid")) prefs.getString("wifi_ssid", currentWifiSsid, sizeof(currentWifiSsid));
  if (prefs.isKey("wifi_pass")) prefs.getString("wifi_pass", currentWifiPass, sizeof(currentWifiPass));
  prefs.end();
  Serial.printf("📦 Loaded from EEPROM/NVS: Drive=%d, TurnL=%d, TurnR=%d, StallInt=%lums, Thresh=%.1fcm, Gain=%d, HeadTol=%.1f°, LaptopIP=%s, SSID='%s'\n",
                defaultDriveSpeed, turnLeftSpeed, turnRightSpeed, stallCheckIntervalMs, obstacleThresholdCm, camSteerGain, headingToleranceDeg, currentLaptopIp, currentWifiSsid);
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
    if (v >= 40 && v <= 255) defaultDriveSpeed = v;
  }
  if (server.hasArg("turn_left_speed")) {
    int v = server.arg("turn_left_speed").toInt();
    if (v >= 50 && v <= 255) turnLeftSpeed = v;
  }
  if (server.hasArg("turn_right_speed")) {
    int v = server.arg("turn_right_speed").toInt();
    if (v >= 50 && v <= 255) turnRightSpeed = v;
  }
  if (server.hasArg("turn_speed") && !server.hasArg("turn_left_speed") && !server.hasArg("turn_right_speed")) {
    int v = server.arg("turn_speed").toInt();
    if (v >= 50 && v <= 255) {
      turnLeftSpeed = v;
      turnRightSpeed = v;
    }
  }
  if (server.hasArg("threshold")) {
    float v = server.arg("threshold").toFloat();
    if (v >= 10.0f && v <= 200.0f) obstacleThresholdCm = v;
  }
  if (server.hasArg("gain")) {
    int v = server.arg("gain").toInt();
    if (v >= 1 && v <= 10) camSteerGain = v;
  }
  if (server.hasArg("tolerance")) {
    float v = server.arg("tolerance").toFloat();
    if (v >= 1.0f && v <= 45.0f) headingToleranceDeg = v;
  }
  if (server.hasArg("stall_interval")) {
    int v = server.arg("stall_interval").toInt();
    if (v >= 100 && v <= 5000) stallCheckIntervalMs = (unsigned long)v;
  }

  if (server.hasArg("laptop_ip")) {
    String lip = server.arg("laptop_ip");
    lip.trim();
    if (lip.length() >= 7 && strncmp(currentLaptopIp, lip.c_str(), sizeof(currentLaptopIp)) != 0) {
      strncpy(currentLaptopIp, lip.c_str(), sizeof(currentLaptopIp));
      netChanged = true;
    }
  }
  if (server.hasArg("wifi_ssid")) {
    String wssid = server.arg("wifi_ssid");
    wssid.trim();
    if (wssid.length() > 0 && strncmp(currentWifiSsid, wssid.c_str(), sizeof(currentWifiSsid)) != 0) {
      strncpy(currentWifiSsid, wssid.c_str(), sizeof(currentWifiSsid));
      netChanged = true;
      wifiChanged = true;
    }
  }
  if (server.hasArg("wifi_pass")) {
    String wpass = server.arg("wifi_pass");
    wpass.trim();
    if (wpass.length() > 0 && strncmp(currentWifiPass, wpass.c_str(), sizeof(currentWifiPass)) != 0) {
      strncpy(currentWifiPass, wpass.c_str(), sizeof(currentWifiPass));
      netChanged = true;
      wifiChanged = true;
    }
  }

  saveSettings(); // Persist changes immediately to flash EEPROM/NVS

  if (netChanged) {
    broadcastMode(1); // Broadcast full config sync to ESP32-CAM via UDP!
    char netLog[128];
    snprintf(netLog, sizeof(netLog), "📡 Config synced to Camera: Laptop IP=%s, Wi-Fi='%s'", currentLaptopIp, currentWifiSsid);
    addLog(netLog);
    if (wifiChanged) {
      Serial.printf("🔄 Connecting Main ESP32 to new Wi-Fi: '%s'...\n", currentWifiSsid);
      WiFi.disconnect();
      WiFi.begin(currentWifiSsid, currentWifiPass);
    }
  } else {
    char logBuf[128];
    snprintf(logBuf, sizeof(logBuf), "⚙️ Settings updated: Drive=%d TurnL=%d TurnR=%d Thresh=%.0fcm Gain=%d Tol=%.1f°",
             defaultDriveSpeed, turnLeftSpeed, turnRightSpeed, obstacleThresholdCm, camSteerGain, headingToleranceDeg);
    addLog(logBuf);
  }

  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// ======================== SETUP ==============================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector to prevent voltage dip reboots
  Serial.begin(115200);
  delay(1000);
  Serial.println(
      F("\n============================================================"));
  Serial.println(
      F("   AUTO WASTE BIN V2 — MAIN ESP32 NAVIGATION NODE          "));
  Serial.println(
      F("============================================================"));

  loadSettings(); // Load saved speeds, threshold, gain, and tolerance from EEPROM/NVS

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

  // Configure Wi-Fi in pure Station (STA) Mode on router to eliminate SoftAP radio overhead:
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
                  currentWifiSsid, WiFi.localIP().toString().c_str(), WiFi.channel());
  } else {
    Serial.println(F("\n⚠️ Wi-Fi router unreachable — activating emergency SoftAP fallback."));
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("⚠️ Emergency SoftAP active: http://%s\n", WiFi.softAPIP().toString().c_str());
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
  Serial.printf("✅ UDP listening on port %d (Target: %d)\n", UDP_MAIN_RX_PORT, UDP_CAM_RX_PORT);

  lastSampleUs = micros();
  lastManualCmdMs = millis();

  delay(500);
  broadcastMode();

  char readyMsg[128];
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(readyMsg, sizeof(readyMsg), "🚀 System ready! Web Dashboard: http://autowaste.local (or http://%s)", WiFi.localIP().toString().c_str());
  } else {
    snprintf(readyMsg, sizeof(readyMsg), "🚀 System ready! SoftAP: http://autowaste.local (or http://%s)", WiFi.softAPIP().toString().c_str());
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

    if (currentMode == MODE_MANUAL && strcmp(currentMotorState, "FORWARD") == 0 &&
        latestDistanceCm > 0 && latestDistanceCm < obstacleThresholdCm) {
      stopMotors();
      char obsBuf[64];
      snprintf(obsBuf, sizeof(obsBuf), "🚨 [MANUAL SAFETY] Obstacle at %.1f cm!", latestDistanceCm);
      addLog(obsBuf);
    }
  }

  if (currentMode == MODE_MANUAL && strcmp(currentMotorState, "STOPPED") != 0 &&
      millis() - lastManualCmdMs > MANUAL_TIMEOUT_MS) {
    stopMotors();
  }

  processClassroomAuto();
  processCamGuided();
}
