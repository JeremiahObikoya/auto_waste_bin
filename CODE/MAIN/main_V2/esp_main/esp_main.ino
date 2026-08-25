/*
 * =============================================================================
 * AUTO WASTE BIN V2 — MAIN ESP32 NAVIGATION NODE
 * =============================================================================
 * Board   : ESP32 Dev Module (30-pin or 38-pin)
 * Role    : Motor control, IMU heading, ultrasonic safety, SoftAP web dashboard,
 *           and ESP-NOW interface with the ESP32-CAM Vision Node.
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
 *   1 MODE_AUTONOMOUS      – Legacy obstacle-avoidance state machine
 *   2 MODE_CAM_GUIDED      – ESP-NOW driven by ESP32-CAM Vision Node
 *
 * Required Libraries:
 *   Adafruit MPU6050, Adafruit Unified Sensor, ESP32Servo
 *
 * ⚠️  Fill in YOUR credentials in secrets.h before flashing!
 * =============================================================================
 */

#include "secrets.h"              // ← Gitignored credentials
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Wire.h>

// ======================== PIN DEFINITIONS ====================================
#define PIN_ENA            25
#define PIN_IN1            26
#define PIN_IN2            27
#define PIN_ENB            13
#define PIN_IN3            14
#define PIN_IN4            19
#define SERVO_ULTRASONIC      32
#define SERVO_ULTRASONIC_PIN  32   // Alias for convenience
#define SERVO_CAM_PIN         33   // Camera pan servo — on THIS ESP32
#define TRIG_PIN               4
#define ECHO_PIN              35   // Input-only GPIO
#define I2C_SDA               21
#define I2C_SCL               22

// Camera servo pan angles for scan cycle: Front/Center (90°), Left (45°), Right (135°)
#define CAM_ANGLE_CENTER      90   // Center / Front
#define CAM_ANGLE_LEFT        45   // Left pan
#define CAM_ANGLE_RIGHT      135   // Right pan

// ======================== OBJECTS & CONSTANTS ================================
WebServer        server(80);
Adafruit_MPU6050 mpu;
Servo            ultrasonicServo;
Servo            camServo;

const float SOUND_SPEED = 0.0343f;   // cm / µs

// ======================== SYSTEM MODES =======================================
enum SystemMode : uint8_t {
  MODE_MANUAL     = 0,
  MODE_AUTONOMOUS = 1,   // Legacy obstacle-avoidance
  MODE_CAM_GUIDED = 2    // ESP-NOW vision-guided
};
SystemMode currentMode = MODE_MANUAL;

// ======================== GLOBAL SENSOR / MOTOR STATE ========================
float   latestDistanceCm  = -1.0f;
String  currentMotorState = "STOPPED";
int     currentSpeedA     = 0;
int     currentSpeedB     = 0;
int     ultrasonicPos     = 90;

// ======================== SYSTEM PARAMETERS ==================================
float obstacleThresholdCm = 50.0f;   // Safety stop distance (cm)
int   defaultDriveSpeed   = 110;
int   defaultTurnSpeed    = 80;
int   camSteerGain        = 2;       // Proportional gain for curve-steering

// ======================== MPU6050 ============================================
bool  mpuConnected        = false;
float gyroZ_offset        = 0.0f;
float currentYaw          = 0.0f;
unsigned long lastSampleUs = 0;
sensors_event_t accelEv, gyroEv, tempEv;

// ======================== SERVO JITTER FIX ===================================
int ultrasonicAngle  = 90;
int camAngleWritten  = CAM_ANGLE_CENTER;
unsigned long lastServoWriteMs    = 0;
unsigned long lastCamServoWriteMs = 0;
#define SERVO_SETTLE_MS 15

void writeUltrasonicServo(int angle) {
  angle = constrain(angle, 0, 180);
  if (angle != ultrasonicAngle && (millis() - lastServoWriteMs >= SERVO_SETTLE_MS)) {
    ultrasonicAngle  = angle;
    ultrasonicPos    = angle;
    ultrasonicServo.write(angle);
    lastServoWriteMs = millis();
  }
}

void writeCamServo(int angle) {
  angle = constrain(angle, 0, 180);
  if (angle != camAngleWritten && (millis() - lastCamServoWriteMs >= SERVO_SETTLE_MS)) {
    camAngleWritten     = angle;
    camServo.write(angle);
    lastCamServoWriteMs = millis();
  }
}

// ======================== DECISION LOG =======================================
#define MAX_LOGS 25
String decisionLogs[MAX_LOGS];
int    logCount = 0;

void addLog(const String &msg) {
  unsigned long s = millis() / 1000;
  char buf[12];
  snprintf(buf, sizeof(buf), "[%02lu:%02lu] ", s / 60, s % 60);
  String entry = String(buf) + msg;
  Serial.println(entry);
  if (logCount < MAX_LOGS) {
    decisionLogs[logCount++] = entry;
  } else {
    for (int i = 0; i < MAX_LOGS - 1; i++) decisionLogs[i] = decisionLogs[i + 1];
    decisionLogs[MAX_LOGS - 1] = entry;
  }
}

// ======================== MOTOR FUNCTIONS ====================================
void setMotorSpeeds(int sA, int sB) {
  currentSpeedA = constrain(sA, 0, 255);
  currentSpeedB = constrain(sB, 0, 255);
  analogWrite(PIN_ENA, currentSpeedA);
  analogWrite(PIN_ENB, currentSpeedB);
}

void stopMotors() {
  currentMotorState = "STOPPED";
  digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(0, 0);
}

void moveForward(int spd = 0) {
  if (spd == 0) spd = defaultDriveSpeed;
  currentMotorState = "FORWARD";
  digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
  setMotorSpeeds(spd, spd);
}

void moveBackward(int spd = 0) {
  if (spd == 0) spd = defaultDriveSpeed;
  currentMotorState = "BACKWARD";
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(spd, spd);
}

void turnLeft(int spd = 0) {
  if (spd == 0) spd = defaultTurnSpeed;
  currentMotorState = "LEFT";
  // Left motor reverse, Right motor forward → turns left
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
  setMotorSpeeds(spd, spd);
}

void turnRight(int spd = 0) {
  if (spd == 0) spd = defaultTurnSpeed;
  currentMotorState = "RIGHT";
  // Left motor forward, Right motor reverse → turns right
  digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(spd, spd);
}

/**
 * @brief Curve-steer toward target using differential motor speed.
 *        angleOffset > 0 = person is to the right → speed up left motor.
 *        angleOffset < 0 = person is to the left  → speed up right motor.
 */
void curveSteer(int8_t angleOffset) {
  currentMotorState = "TRACKING";
  int left  = constrain(defaultDriveSpeed + angleOffset * camSteerGain, 40, 255);
  int right = constrain(defaultDriveSpeed - angleOffset * camSteerGain, 40, 255);
  digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, left);
  analogWrite(PIN_ENB, right);
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

void updateMPU() {
  if (!mpuConnected) return;
  unsigned long now = micros();
  float dt = (now - lastSampleUs) / 1000000.0f;
  lastSampleUs = now;
  mpu.getEvent(&accelEv, &gyroEv, &tempEv);
  float gz = (gyroEv.gyro.z - gyroZ_offset) * (180.0f / PI);
  if (fabsf(gz) > 0.6f) currentYaw += gz * dt;
}

void calibrateMPU() {
  if (!mpuConnected) return;
  addLog("🔧 Calibrating MPU6050 — keep still...");
  float sum = 0;
  for (int i = 0; i < 300; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sum += g.gyro.z;
    delay(3);
  }
  gyroZ_offset = sum / 300.0f;
  addLog("✅ MPU6050 calibrated. Offset: " + String(gyroZ_offset * (180.0f / PI), 3) + " °/s");
}

// ======================== ESP-NOW ============================================
/** Packet received from ESP32-CAM → this ESP32 */
struct CamPacket {
  uint8_t state;         // 0=SCANNING, 1=TRACKING, 2=INTERACTION, 3=RELEASE
  int8_t  angleOffset;
};

/** Packet sent from this ESP32 → ESP32-CAM */
struct MainPacket {
  uint8_t systemMode;    // 0=MANUAL, 1=AUTONOMOUS_LEGACY, 2=CAM_GUIDED
};

uint8_t      camMAC[] = ESPCAM_MAC;
esp_now_peer_info_t camPeer;

volatile CamPacket lastCamPkt = {0, 0};
volatile bool      newCamPkt  = false;

// ESP32 Arduino core v3.x (IDF 5.x) changed the ESP-NOW callback signatures:
//   Send : const wifi_tx_info_t*   (previously const uint8_t*)
//   Recv : const esp_now_recv_info_t* (previously const uint8_t*)
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Serial.printf("[ESP-NOW TX] %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(CamPacket)) {
    memcpy((void*)&lastCamPkt, data, sizeof(CamPacket));
    newCamPkt = true;
  }
}

/** Broadcast system mode/pause state to the ESP32-CAM.
 *  systemMode values sent:
 *    0 = MANUAL        (CAM suspends Gemini)
 *    1 = AUTONOMOUS    (CAM suspends Gemini)
 *    2 = CAM_GUIDED    (CAM runs Gemini)
 *    3 = CAM_PAUSED    (CAM suspends Gemini, keeps state)
 */
bool camPaused = false;

void broadcastMode() {
  MainPacket pkt;
  if (camPaused && currentMode == MODE_CAM_GUIDED)
    pkt.systemMode = 3;   // PAUSED — CAM stops Gemini
  else
    pkt.systemMode = (uint8_t)currentMode;
  esp_now_send(camMAC, (uint8_t *)&pkt, sizeof(pkt));
  Serial.printf("[ESP-NOW TX] systemMode=%d\n", pkt.systemMode);
}

// ======================== CAM-GUIDED STATE MACHINE ===========================
// Step hold duration at each servo position (Center, Left, Right) to give Gemini time to scan
#define STEP_HOLD_MS      5000   // 5 s per servo position (Center -> Left -> Right)
#define IDLE_DURATION_MS 10000   // 10 s idle after full 360° scan (front 2x + rear 2x)

enum FacingOrientation : uint8_t {
  ORIENTATION_FRONT = 0,
  ORIENTATION_REAR  = 1
};

enum CamGuidedSub {
  CG_IDLE,                 // Initial entry / reset
  CG_SCANNING_CYCLE,       // Stepping camera servo: Center (90°) → Left (45°) → Right (135°)
  CG_TURN_180_REAR,        // Robot rotating 180° with MPU6050 to face rear
  CG_TURN_180_FRONT,       // Robot rotating 180° with MPU6050 back to front
  CG_IDLE_WAIT,            // 10 s idle rest period
  CG_TRACKING,             // Person detected — moving forward & curve-steering
  CG_AVOID_OBSTACLE,       // Obstacle in path while tracking — scanning left/right and steering around
  CG_HALT,                 // Reached person — halted waiting for drop-off
  CG_BACKOFF_REVERSE,      // Reversing 2 s after drop confirmed
  CG_BACKOFF_ROTATE        // 180° departure turn after back-off
};

CamGuidedSub      cgSub            = CG_IDLE;
FacingOrientation robotOrientation = ORIENTATION_FRONT;
int               currentCycle     = 1;  // 1 or 2
int               currentStep      = 0;  // 0=CENTER (90°), 1=LEFT (45°), 2=RIGHT (135°)
unsigned long     stepStartMs      = 0;
unsigned long     idleStartMs      = 0;
unsigned long     cgReverseStart   = 0;
unsigned long     haltStartMs      = 0;
int               avoidStep        = 0;
unsigned long     avoidTimer       = 0;
float             leftAvoidDist    = -1.0f;
float             rightAvoidDist   = -1.0f;
float             cgRotateTarget   = 0.0f;
String            camStateStr      = "IDLE";
int8_t            camAngleOffset   = 0;

const char* getStepName(int step) {
  if (step == 0) return "CENTER (90°)";
  if (step == 1) return "LEFT (45°)";
  if (step == 2) return "RIGHT (135°)";
  return "--";
}

void processCamGuided() {
  if (currentMode != MODE_CAM_GUIDED) return;
  if (camPaused) { stopMotors(); return; }  // Paused — hold motors still

  // Cache volatile packet from ESP32-CAM
  CamPacket pkt;
  noInterrupts();
  memcpy(&pkt, (const void*)&lastCamPkt, sizeof(CamPacket));
  newCamPkt = false;
  interrupts();

  camAngleOffset = pkt.angleOffset;
  unsigned long now = millis();

  // ── Back-off Reverse (2 s) ───────────────────────────────────────────────
  if (cgSub == CG_BACKOFF_REVERSE) {
    camStateStr = "BACKING OFF";
    if (now - cgReverseStart >= 2000) {
      stopMotors();
      cgRotateTarget = currentYaw + 180.0f;
      addLog("🔄 Reverse complete — starting 180° departure turn...");
      cgSub = CG_BACKOFF_ROTATE;
      turnRight();
    }
    return;
  }

  // ── Back-off 180° Departure Turn ──────────────────────────────────────────
  if (cgSub == CG_BACKOFF_ROTATE) {
    camStateStr = "ROTATING 180°";
    float err = cgRotateTarget - currentYaw;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    if (fabsf(err) <= 8.0f || !mpuConnected) {
      stopMotors();
      addLog("✅ Departure turn complete — ready for next scan.");
      writeCamServo(CAM_ANGLE_CENTER);
      cgSub = CG_IDLE;
      camStateStr = "IDLE";
    }
    return;
  }

  // ── 180° MPU Turn to Face REAR ────────────────────────────────────────────
  if (cgSub == CG_TURN_180_REAR) {
    camStateStr = "ROTATING 180° TO REAR";
    float err = cgRotateTarget - currentYaw;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    if (fabsf(err) <= 8.0f || !mpuConnected) {
      stopMotors();
      robotOrientation = ORIENTATION_REAR;
      currentCycle     = 1;
      currentStep      = 0;
      writeCamServo(CAM_ANGLE_CENTER);
      stepStartMs      = millis();
      addLog("✅ 180° turn complete — robot is now facing REAR.");
      addLog("📡 [REAR] Cycle 1/2 — checking CENTER (90°)...");
      cgSub            = CG_SCANNING_CYCLE;
      camStateStr      = "SCANNING REAR (1/2 CENTER)";
    }
    return;
  }

  // ── 180° MPU Turn to Face FRONT ───────────────────────────────────────────
  if (cgSub == CG_TURN_180_FRONT) {
    camStateStr = "ROTATING 180° TO FRONT";
    float err = cgRotateTarget - currentYaw;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    if (fabsf(err) <= 8.0f || !mpuConnected) {
      stopMotors();
      robotOrientation = ORIENTATION_FRONT;
      idleStartMs      = millis();
      addLog("✅ 180° turn complete — robot is now facing FRONT.");
      addLog("💤 No targets found in 360°. Entering 10-second Idle Mode...");
      cgSub            = CG_IDLE_WAIT;
      camStateStr      = "IDLE — WAITING (10s)";
    }
    return;
  }

  // ── Target Detected during Scanning Cycle ─────────────────────────────────
  // Because the entire robot turns 180° when checking the rear, the robot is
  // ALREADY FACING the detected person in both FRONT and REAR orientations!
  if (cgSub == CG_SCANNING_CYCLE && pkt.state == 1) {
    String orientStr = (robotOrientation == ORIENTATION_FRONT) ? "FRONT" : "REAR";
    addLog("🎯 Target detected facing " + orientStr + "! Moving forward to approach...");
    writeCamServo(CAM_ANGLE_CENTER);  // Re-centre camera servo
    cgSub = CG_TRACKING;
    camStateStr = "TRACKING (" + orientStr + ")";
    curveSteer(pkt.angleOffset);
    return;
  }

  // ── State Machine Processing ──────────────────────────────────────────────
  switch (cgSub) {

    case CG_IDLE:
      stopMotors();
      robotOrientation = ORIENTATION_FRONT;
      currentCycle     = 1;
      currentStep      = 0;
      writeCamServo(CAM_ANGLE_CENTER);
      stepStartMs      = now;
      addLog("📡 [FRONT] Autonomous scan started — Cycle 1/2: checking CENTER (90°)...");
      cgSub            = CG_SCANNING_CYCLE;
      camStateStr      = "SCANNING FRONT (1/2 CENTER)";
      break;

    case CG_SCANNING_CYCLE: {
      stopMotors();   // Robot remains strictly stationary while scanning
      String orient = (robotOrientation == ORIENTATION_FRONT) ? "FRONT" : "REAR";
      camStateStr   = "SCANNING " + orient + " (" + String(currentCycle) + "/2 " + String(getStepName(currentStep)) + ")";

      // Step hold timeout: move camera servo to next position (Center -> Left -> Right)
      if (now - stepStartMs >= STEP_HOLD_MS) {
        currentStep++;

        if (currentStep == 1) {
          writeCamServo(CAM_ANGLE_LEFT);
          stepStartMs = millis();
          addLog("🔍 [" + orient + "] Cycle " + String(currentCycle) + "/2 — checking LEFT (45°)...");
        }
        else if (currentStep == 2) {
          writeCamServo(CAM_ANGLE_RIGHT);
          stepStartMs = millis();
          addLog("🔍 [" + orient + "] Cycle " + String(currentCycle) + "/2 — checking RIGHT (135°)...");
        }
        else {
          // Completed one full sweep (Center, Left, Right)
          currentStep = 0;
          writeCamServo(CAM_ANGLE_CENTER);
          currentCycle++;

          if (currentCycle <= 2) {
            // Start cycle 2 in this same orientation
            stepStartMs = millis();
            addLog("🔍 [" + orient + "] Starting Scan Cycle 2/2 — checking CENTER (90°)...");
          } else {
            // Completed 2 full cycles in this orientation with no target!
            if (robotOrientation == ORIENTATION_FRONT) {
              addLog("❌ 2 cycles completed facing FRONT — no one detected.");
              addLog("🔄 Rotating robot 180° with MPU6050 to check REAR...");
              cgRotateTarget = currentYaw + 180.0f;
              cgSub          = CG_TURN_180_REAR;
              camStateStr    = "ROTATING 180° TO REAR";
              turnRight();
            } else {
              addLog("❌ 2 cycles completed facing REAR — no one detected.");
              addLog("🔄 Rotating robot 180° with MPU6050 back to FRONT...");
              cgRotateTarget = currentYaw + 180.0f;
              cgSub          = CG_TURN_180_FRONT;
              camStateStr    = "ROTATING 180° TO FRONT";
              turnRight();
            }
          }
        }
      }
      break;
    }

    case CG_IDLE_WAIT: {
      stopMotors();
      // Countdown log every 3 seconds
      static unsigned long lastIdleLogMs = 0;
      if (now - lastIdleLogMs >= 3000) {
        lastIdleLogMs = now;
        long remaining = (IDLE_DURATION_MS - (now - idleStartMs)) / 1000;
        if (remaining > 0)
          addLog("⏳ Idle — resuming scan in " + String(remaining) + " s...");
      }
      if (now - idleStartMs >= IDLE_DURATION_MS) {
        addLog("⏰ 10s Idle complete. Restarting Autonomous Scan routine...");
        cgSub = CG_IDLE;
      }
      break;
    }

    case CG_TRACKING:
      camStateStr = "TRACKING";
      if (pkt.state == 1) {
        // Obstacle detected in path while approaching person -> swerve around it
        if (latestDistanceCm > 0 && latestDistanceCm < obstacleThresholdCm) {
          stopMotors();
          avoidStep  = 0;
          avoidTimer = now;
          writeUltrasonicServo(150);  // Look left
          addLog("🚧 Obstacle at " + String(latestDistanceCm, 1) + " cm! Scanning path to navigate around...");
          cgSub = CG_AVOID_OBSTACLE;
          return;
        }
        curveSteer(pkt.angleOffset);
      } else if (pkt.state == 2) {  // INTERACTION (reached person)
        stopMotors();
        cgSub = CG_HALT;
        haltStartMs = now;
        writeUltrasonicServo(90);
        addLog("🗑️ Target reached! Waiting for waste drop-off...");
        camStateStr = "HALTED — DROP OFF";
      } else if (pkt.state == 0) {  // Lost target
        addLog("⚠️ Target lost — restarting autonomous scan.");
        stopMotors();
        writeUltrasonicServo(90);
        cgSub = CG_IDLE;
      }
      break;

    case CG_AVOID_OBSTACLE:
      camStateStr = "AVOIDING OBSTACLE";
      if (pkt.state == 0) {
        addLog("⚠️ Target lost during avoidance — restarting scan.");
        stopMotors();
        writeUltrasonicServo(90);
        cgSub = CG_IDLE;
        return;
      }
      switch (avoidStep) {
        case 0: // Scan Left (wait 300ms)
          if (now - avoidTimer >= 300) {
            leftAvoidDist = readUltrasonic();
            writeUltrasonicServo(30);  // Look right
            avoidTimer = now;
            avoidStep  = 1;
          }
          break;
        case 1: // Scan Right (wait 300ms)
          if (now - avoidTimer >= 300) {
            rightAvoidDist = readUltrasonic();
            writeUltrasonicServo(90);  // Re-center sonar
            avoidTimer = now;
            avoidStep  = 2;
          }
          break;
        case 2: { // Decide path
          float l = (leftAvoidDist  <= 0) ? 999.0f : leftAvoidDist;
          float r = (rightAvoidDist <= 0) ? 999.0f : rightAvoidDist;
          if (l > r && l > obstacleThresholdCm) {
            addLog("💡 Swerving LEFT around obstacle (" + String(l, 1) + " cm clear)");
            turnLeft(defaultTurnSpeed);
            avoidTimer = now;
            avoidStep  = 3;
          } else if (r >= l && r > obstacleThresholdCm) {
            addLog("💡 Swerving RIGHT around obstacle (" + String(r, 1) + " cm clear)");
            turnRight(defaultTurnSpeed);
            avoidTimer = now;
            avoidStep  = 3;
          } else {
            addLog("⚠️ Both sides narrow — backing up...");
            moveBackward(defaultTurnSpeed);
            avoidTimer = now;
            avoidStep  = 3;
          }
          break;
        }
        case 3: // Complete swerve (wait 400ms)
          if (now - avoidTimer >= 400) {
            stopMotors();
            addLog("✅ Path cleared — resuming camera tracking.");
            cgSub = CG_TRACKING;
          }
          break;
      }
      break;

    case CG_HALT:
      camStateStr = "HALTED — DROP OFF";
      stopMotors();
      if (pkt.state == 3) {  // RELEASE (waste drop confirmed)
        addLog("⬅️ Drop confirmed! Reversing 2 seconds...");
        moveBackward();
        cgReverseStart = millis();
        cgSub          = CG_BACKOFF_REVERSE;
        camStateStr    = "BACKING OFF";
      } else if (pkt.state == 0 || (now - haltStartMs >= 15000)) {
        // User walked away or 15s timeout
        addLog("⏱️ Drop-off completed / user left — resuming scan.");
        stopMotors();
        writeUltrasonicServo(90);
        cgSub = CG_IDLE;
      }
      break;

    default:
      break;
  }
}

// ======================== LEGACY AUTONOMOUS STATE MACHINE ====================
enum AutoState {
  AUTO_IDLE, AUTO_DRIVE, AUTO_OBSTACLE_DETECTED,
  AUTO_BACKUP, AUTO_SCAN_LEFT, AUTO_SCAN_RIGHT,
  AUTO_DECIDE, AUTO_TURNING
};
AutoState autoState = AUTO_IDLE;
float leftScanDist  = -1.0f;
float rightScanDist = -1.0f;
unsigned long autoTimer = 0;
int   autoTurnMs        = 650;

void processLegacyAuto() {
  if (currentMode != MODE_AUTONOMOUS) return;
  unsigned long now = millis();
  switch (autoState) {
    case AUTO_IDLE:
      addLog("🤖 Obstacle-Avoidance started.");
      writeUltrasonicServo(90);
      autoState = AUTO_DRIVE;
      break;
    case AUTO_DRIVE:
      writeUltrasonicServo(90);
      moveForward();
      if (latestDistanceCm > 0 && latestDistanceCm <= obstacleThresholdCm) {
        stopMotors();
        addLog("🚨 OBSTACLE at " + String(latestDistanceCm, 1) + " cm!");
        autoTimer = now;
        autoState = AUTO_OBSTACLE_DETECTED;
      }
      break;
    case AUTO_OBSTACLE_DETECTED:
      if (now - autoTimer >= 200) {
        moveBackward(); autoTimer = now; autoState = AUTO_BACKUP;
        addLog("⬅️ Reversing...");
      }
      break;
    case AUTO_BACKUP:
      if (now - autoTimer >= 500) {
        stopMotors();
        writeUltrasonicServo(160);
        autoTimer = now; autoState = AUTO_SCAN_LEFT;
        addLog("🔍 Scanning LEFT...");
      }
      break;
    case AUTO_SCAN_LEFT:
      if (now - autoTimer >= 350) {
        leftScanDist = readUltrasonic();
        addLog("🔍 Left: " + (leftScanDist > 0 ? String(leftScanDist,1)+" cm" : "Clear"));
        writeUltrasonicServo(20);
        autoTimer = now; autoState = AUTO_SCAN_RIGHT;
      }
      break;
    case AUTO_SCAN_RIGHT:
      if (now - autoTimer >= 350) {
        rightScanDist = readUltrasonic();
        addLog("🔍 Right: " + (rightScanDist > 0 ? String(rightScanDist,1)+" cm" : "Clear"));
        writeUltrasonicServo(90);
        autoState = AUTO_DECIDE;
      }
      break;
    case AUTO_DECIDE: {
      float l = (leftScanDist  <= 0) ? 999.0f : leftScanDist;
      float r = (rightScanDist <= 0) ? 999.0f : rightScanDist;
      if (l > r && l > obstacleThresholdCm) {
        addLog("💡 Turning LEFT (" + String(l,1) + " vs " + String(r,1) + " cm)");
        turnLeft(); autoTurnMs = 650;
      } else if (r >= l && r > obstacleThresholdCm) {
        addLog("💡 Turning RIGHT (" + String(r,1) + " vs " + String(l,1) + " cm)");
        turnRight(); autoTurnMs = 650;
      } else {
        addLog("⚠️ Both blocked — U-Turn!");
        turnRight(); autoTurnMs = 1200;
      }
      autoTimer = now; autoState = AUTO_TURNING;
      break;
    }
    case AUTO_TURNING:
      if (now - autoTimer >= (unsigned long)autoTurnMs) {
        stopMotors();
        addLog("✅ Turn done — resuming drive.");
        autoState = AUTO_DRIVE;
      }
      break;
  }
}

// ======================== MANUAL MODE TIMEOUT ================================
unsigned long lastManualCmdMs = 0;
#define MANUAL_TIMEOUT_MS 2000

// ======================== HTML DASHBOARD =====================================
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
    .cam-state-badge{padding:6px 0;border-radius:8px;text-align:center;font-size:.82rem;font-weight:bold}
    .cs-scan{background:rgba(210,153,34,.15);border:1px solid var(--orange);color:var(--orange)}
    .cs-track{background:rgba(47,129,247,.15);border:1px solid var(--accent);color:var(--accent)}
    .cs-inter{background:rgba(35,134,54,.15);border:1px solid var(--green);color:#2ea043}
    .cs-rel{background:rgba(218,54,51,.15);border:1px solid var(--red);color:var(--red)}
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
    <button class="tab-btn" id="tab-auto-btn"   onclick="switchMode('auto')">🤖 Obstacle Avoidance</button>
    <button class="tab-btn" id="tab-cam-btn"    onclick="switchMode('cam')">📷 CAM Guided Auto</button>
  </div>

  <!-- Telemetry Bar -->
  <div class="grid">
    <div class="stat-box"><div class="stat-label">System Mode</div>
      <div class="stat-val" id="tele-mode"><span class="badge badge-manual">MANUAL</span></div></div>
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
        <p style="text-align:center;font-size:.78rem;color:var(--muted)">Keyboard: W A S D to move · Space to stop</p>
      </div>
      <div class="card">
        <h2>📡 Ultrasonic Servo (GPIO 32)</h2>
        <div class="control-group">
          <div class="slider-card">
            <div class="slider-header"><span>Servo Angle</span><span id="us-servo-val">90°</span></div>
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
    <div class="card" style="margin-top:14px">
      <h2>⚙️ System Settings & Speed Tuning</h2>
      <div class="grid" style="grid-template-columns:repeat(auto-fit,minmax(200px,1fr))">
        <div class="slider-card">
          <div class="slider-header"><span>Drive Speed PWM</span><span id="val-drive">110</span></div>
          <input type="range" id="sl-drive" min="50" max="255" value="110" onchange="saveSettings()">
        </div>
        <div class="slider-card">
          <div class="slider-header"><span>Turn Speed PWM</span><span id="val-turn">80</span></div>
          <input type="range" id="sl-turn" min="50" max="255" value="80" onchange="saveSettings()">
        </div>
        <div class="slider-card">
          <div class="slider-header"><span>Obstacle Threshold</span><span id="val-thresh">50 cm</span></div>
          <input type="range" id="sl-thresh" min="10" max="100" value="50" onchange="saveSettings()">
        </div>
        <div class="slider-card">
          <div class="slider-header"><span>CAM Steer Gain</span><span id="val-gain">2</span></div>
          <input type="range" id="sl-gain" min="1" max="5" value="2" onchange="saveSettings()">
        </div>
      </div>
      <div style="margin-top:11px;display:flex;justify-content:flex-end;gap:9px">
        <button class="btn-sm" style="background:#21262d" onclick="resetYaw()">🔄 Reset Yaw to 0°</button>
      </div>
    </div>
  </div>

  <!-- TAB 2: LEGACY AUTONOMOUS -->
  <div id="auto-view" style="display:none">
    <div class="card">
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:11px">
        <h2>🧠 Obstacle-Avoidance Decision Log</h2>
        <button class="btn-sm" style="background:var(--red)" onclick="switchMode('manual')">🛑 Stop Autonomous</button>
      </div>
      <div class="terminal" id="auto-terminal"><div class="log-entry">Waiting for autonomous telemetry...</div></div>
    </div>
  </div>

  <!-- TAB 3: CAM GUIDED AUTONOMOUS -->
  <div id="cam-view" style="display:none">
    <div class="card">
      <!-- Header row: title + control buttons -->
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:11px;flex-wrap:wrap;gap:8px">
        <h2 style="margin-bottom:0;border:none;padding:0">📷 CAM Guided — Vision State</h2>
        <div style="display:flex;gap:8px;align-items:center">
          <span id="cam-paused-badge" style="display:none;padding:4px 10px;border-radius:12px;font-size:.78rem;font-weight:bold;background:rgba(218,54,51,.2);color:#da3633;border:1px solid #da3633">⏸ PAUSED</span>
          <button id="btn-cam-pause"  class="btn-sm" style="background:#d29922;color:#000" onclick="camPauseToggle(true)">⏸ Pause</button>
          <button id="btn-cam-resume" class="btn-sm" style="display:none;background:#238636" onclick="camPauseToggle(false)">▶ Resume</button>
          <button class="btn-sm" style="background:var(--red)" onclick="switchMode('manual')">🛑 Exit CAM Mode</button>
        </div>
      </div>
      <!-- Status cards -->
      <div class="cam-status-grid" style="grid-template-columns:repeat(auto-fit,minmax(130px,1fr))">
        <div class="stat-box"><div class="stat-label">Robot Sub-State</div>
          <div class="stat-val" id="cam-sub-val" style="font-size:1.05rem">IDLE</div></div>
        <div class="stat-box"><div class="stat-label">Robot Facing</div>
          <div class="stat-val" id="cam-facing-val">FRONT</div></div>
        <div class="stat-box"><div class="stat-label">Camera Pan</div>
          <div class="stat-val" id="cam-pan-val" style="font-size:1rem">CENTER (90°)</div></div>
        <div class="stat-box"><div class="stat-label">Scan Cycle</div>
          <div class="stat-val" id="cam-cycle-val">1 of 2</div></div>
        <div class="stat-box"><div class="stat-label">Angle Offset</div>
          <div class="stat-val" id="cam-angle-val">0°</div></div>
      </div>
      <!-- Vision phase indicator bar -->
      <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(110px,1fr));gap:8px;margin-bottom:14px">
        <div class="cam-state-badge cs-scan"  id="ind-scan-front" >📡 SCAN FRONT</div>
        <div class="cam-state-badge cs-scan"  id="ind-rotate"     style="opacity:.3">🔄 ROTATE 180°</div>
        <div class="cam-state-badge cs-scan"  id="ind-scan-rear"  style="opacity:.3">📡 SCAN REAR</div>
        <div class="cam-state-badge cs-track" id="ind-track"      style="opacity:.3">🎯 TRACKING</div>
        <div class="cam-state-badge cs-inter" id="ind-inter"      style="opacity:.3">🗑️ DROP-OFF</div>
      </div>
      <!-- Decision log terminal -->
      <div class="terminal" id="cam-terminal"><div class="log-entry">Activate CAM Guided mode to see live vision logs...</div></div>
    </div>
  </div>
</div>

<script>
  let uiMode = 'manual';
  let camIsPaused = false;

  function switchMode(mode) {
    uiMode = mode;
    // Reset pause state when switching modes
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
    let d=document.getElementById('sl-drive').value,
        t=document.getElementById('sl-turn').value,
        th=document.getElementById('sl-thresh').value,
        g=document.getElementById('sl-gain').value;
    document.getElementById('val-drive').innerText  = d;
    document.getElementById('val-turn').innerText   = t;
    document.getElementById('val-thresh').innerText = th+' cm';
    document.getElementById('val-gain').innerText   = g;
    fetch('/api/settings?drive_speed='+d+'&turn_speed='+t+'&threshold='+th+'&gain='+g);
  }

  function resetYaw() { fetch('/api/cmd?reset_yaw=1'); }

  document.addEventListener('keydown', e => {
    if(uiMode!=='manual'||e.repeat) return;
    if(e.key==='w'||e.key==='W') drv('forward');
    if(e.key==='s'||e.key==='S') drv('backward');
    if(e.key==='a'||e.key==='A') drv('left');
    if(e.key==='d'||e.key==='D') drv('right');
    if(e.key===' ') drv('stop');
  });
  document.addEventListener('keyup', e => {
    if(uiMode!=='manual') return;
    if(['w','W','s','S','a','A','d','D'].includes(e.key)) drv('stop');
  });

  function poll() {
    fetch('/api/telemetry').then(r=>r.json()).then(updateUI).catch(()=>{});
  }

  function updateUI(d) {
    if(!d) return;

    // Mode badge
    let mb='<span class="badge badge-manual">MANUAL</span>';
    if(d.mode==='AUTONOMOUS') mb='<span class="badge badge-auto">AUTONOMOUS</span>';
    if(d.mode==='CAM_GUIDED') mb='<span class="badge badge-cam">CAM GUIDED</span>';
    document.getElementById('tele-mode').innerHTML = mb;
    document.getElementById('tele-dist').innerText  = d.ultrasonic>=0 ? d.ultrasonic.toFixed(1)+' cm' : 'Out of Range';
    document.getElementById('tele-yaw').innerText   = (d.mpu_yaw>=0?'+':'')+d.mpu_yaw.toFixed(1)+'°';
    document.getElementById('tele-motor').innerText = d.motor_state;

    // Sync pause button state from server
    if(d.cam_paused !== undefined) {
      let p = d.cam_paused;
      document.getElementById('btn-cam-pause').style.display   = p ? 'none'  : 'inline-block';
      document.getElementById('btn-cam-resume').style.display  = p ? 'inline-block' : 'none';
      document.getElementById('cam-paused-badge').style.display = p ? 'inline-block' : 'none';
    }

    // CAM sub-state, orientation, pan position, and cycle
    if(d.cam_sub!==undefined) {
      document.getElementById('cam-sub-val').innerText    = d.cam_sub||'IDLE';
      document.getElementById('cam-facing-val').innerText = d.robot_facing||'FRONT';
      document.getElementById('cam-pan-val').innerText    = d.cam_pan||'CENTER (90°)';
      document.getElementById('cam-cycle-val').innerText  = d.scan_cycle||'1 of 2';
      document.getElementById('cam-angle-val').innerText  = (d.cam_angle>=0?'+':'')+d.cam_angle+'°';

      // Highlight correct phase indicator
      let sub = (d.cam_sub || '').toUpperCase();
      document.getElementById('ind-scan-front').style.opacity = (sub.includes('FRONT') && sub.includes('SCAN')) ? '1' : '0.3';
      document.getElementById('ind-rotate').style.opacity     = (sub.includes('ROTAT') || sub.includes('180°'))   ? '1' : '0.3';
      document.getElementById('ind-scan-rear').style.opacity  = (sub.includes('REAR')  && sub.includes('SCAN')) ? '1' : '0.3';
      document.getElementById('ind-track').style.opacity      = sub.includes('TRACK')                              ? '1' : '0.3';
      document.getElementById('ind-inter').style.opacity      = (sub.includes('DROP') || sub.includes('HALT'))    ? '1' : '0.3';
    }

    // Log terminal (CAM tab only gets CAM logs)
    if(d.logs && d.logs.length) {
      let html = d.logs.map(l=>'<div class="log-entry">'+l+'</div>').join('');
      document.getElementById('auto-terminal').innerHTML = html;
      document.getElementById('auto-terminal').scrollTop = 9999;
      document.getElementById('cam-terminal').innerHTML  = html;
      document.getElementById('cam-terminal').scrollTop  = 9999;
    }
  }

  setInterval(poll, 300);
</script>
</body>
</html>
)rawliteral";

// ======================== WEB HANDLERS =======================================
void handleRoot()      { server.send(200, "text/html", INDEX_HTML); }

void handleTelemetry() {
  String modeStr = "MANUAL";
  if (currentMode == MODE_AUTONOMOUS) modeStr = "AUTONOMOUS";
  if (currentMode == MODE_CAM_GUIDED) modeStr = "CAM_GUIDED";

  String orientStr = (robotOrientation == ORIENTATION_FRONT) ? "FRONT" : "REAR";
  String cycleStr  = (cgSub == CG_SCANNING_CYCLE) ? (String(currentCycle) + " of 2") : "--";
  String panStr    = (cgSub == CG_SCANNING_CYCLE) ? String(getStepName(currentStep)) : "CENTER (90°)";

  String json = "{";
  json += "\"mode\":\""          + modeStr                         + "\",";
  json += "\"ultrasonic\":"     + String(latestDistanceCm, 1)      + ",";
  json += "\"mpu_yaw\":"        + String(currentYaw, 2)            + ",";
  json += "\"motor_state\":\""   + currentMotorState               + "\",";
  json += "\"drive_speed\":"    + String(defaultDriveSpeed)        + ",";
  json += "\"turn_speed\":"     + String(defaultTurnSpeed)         + ",";
  json += "\"threshold\":"      + String(obstacleThresholdCm, 1)   + ",";
  json += "\"us_servo\":"       + String(ultrasonicAngle)          + ",";
  json += "\"cam_state\":"      + String(lastCamPkt.state)         + ",";
  json += "\"cam_angle\":"      + String(lastCamPkt.angleOffset)   + ",";
  json += "\"cam_sub\":\""      + camStateStr                      + "\",";
  json += "\"robot_facing\":\"" + orientStr                        + "\",";
  json += "\"scan_cycle\":\""   + cycleStr                         + "\",";
  json += "\"cam_pan\":\""      + panStr                           + "\",";
  json += "\"cam_paused\":"     + String(camPaused ? "true" : "false") + ",";
  json += "\"logs\":[";
  for (int i = 0; i < logCount; i++) {
    json += "\"" + decisionLogs[i] + "\"";
    if (i < logCount - 1) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleCommand() {
  lastManualCmdMs = millis();

  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    if (m == "auto") {
      currentMode = MODE_AUTONOMOUS;
      autoState   = AUTO_IDLE;
      camPaused   = false;
      addLog("🔄 Mode → AUTONOMOUS (Obstacle Avoidance)");
    } else if (m == "cam") {
      currentMode = MODE_CAM_GUIDED;
      cgSub       = CG_IDLE;
      camPaused   = false;
      camStateStr = "IDLE";
      addLog("🔄 Mode → CAM GUIDED (Vision Node)");
    } else {
      currentMode = MODE_MANUAL;
      camPaused   = false;
      stopMotors();
      addLog("🔄 Mode → MANUAL");
    }
    broadcastMode();   // Notify ESP32-CAM of mode change
  }

  // CAM Pause / Resume (stays in CAM tab — does NOT switch to manual)
  if (server.hasArg("cam_pause") && currentMode == MODE_CAM_GUIDED) {
    camPaused = true;
    stopMotors();
    broadcastMode();   // Sends systemMode=3 (PAUSED) to ESP32-CAM
    addLog("⏸ CAM Guided PAUSED by user.");
  }
  if (server.hasArg("cam_resume") && currentMode == MODE_CAM_GUIDED) {
    camPaused = false;
    cgSub     = CG_IDLE;   // Restart scan cycle cleanly
    broadcastMode();       // Sends systemMode=2 (CAM_GUIDED) to ESP32-CAM
    addLog("▶ CAM Guided RESUMED — restarting scan.");
  }

  if (currentMode == MODE_MANUAL && server.hasArg("dir")) {
    String dir = server.arg("dir");
    if (dir == "forward") {
      if (latestDistanceCm > 0 && latestDistanceCm < obstacleThresholdCm) {
        stopMotors();
        addLog("🚨 Obstacle at " + String(latestDistanceCm, 1) + " cm — cannot advance!");
      } else {
        moveForward();
      }
    } else if (dir == "backward") { moveBackward(); }
    else if (dir == "left")       { turnLeft(); }
    else if (dir == "right")      { turnRight(); }
    else if (dir == "stop")       { stopMotors(); }
  }

  if (server.hasArg("us_servo")) {
    writeUltrasonicServo(server.arg("us_servo").toInt());
  }
  if (server.hasArg("reset_yaw")) {
    currentYaw = 0.0f;
    addLog("🔄 Yaw zeroed.");
  }
  handleTelemetry();
}

void handleSettings() {
  if (server.hasArg("drive_speed")) defaultDriveSpeed   = server.arg("drive_speed").toInt();
  if (server.hasArg("turn_speed"))  defaultTurnSpeed    = server.arg("turn_speed").toInt();
  if (server.hasArg("threshold"))   obstacleThresholdCm = server.arg("threshold").toFloat();
  if (server.hasArg("gain"))        camSteerGain        = server.arg("gain").toInt();
  addLog("⚙️ Settings: Drive=" + String(defaultDriveSpeed) +
         " Turn=" + String(defaultTurnSpeed) +
         " Threshold=" + String(obstacleThresholdCm, 0) + "cm" +
         " Gain=" + String(camSteerGain));
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// ======================== SETUP ==============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n============================================================"));
  Serial.println(F("   AUTO WASTE BIN V2 — MAIN ESP32 NAVIGATION NODE          "));
  Serial.println(F("============================================================"));

  // 1. Servos (both on this ESP32) — allocate Timer 2 and 3 to avoid conflicts with motor analogWrite
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  ultrasonicServo.setPeriodHertz(50);
  camServo.setPeriodHertz(50);
  if (ultrasonicServo.attach(SERVO_ULTRASONIC, 500, 2400))
    Serial.printf("✅ Ultrasonic servo on GPIO %d\n", SERVO_ULTRASONIC);
  if (camServo.attach(SERVO_CAM_PIN, 500, 2400))
    Serial.printf("✅ Camera pan servo on GPIO %d\n", SERVO_CAM_PIN);
  delay(30);
  writeUltrasonicServo(90);
  writeCamServo(CAM_ANGLE_CENTER);   // Start camera facing forward

  // 2. Motor pins
  int mPins[] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB};
  for (int p : mPins) pinMode(p, OUTPUT);
  stopMotors();
  Serial.println(F("✅ Motor driver pins configured"));

  // 3. Ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.println(F("✅ Ultrasonic sensor ready"));

  // 4. MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin()) {
    Serial.println(F("⚠️  MPU6050 not found — continuing without IMU."));
  } else {
    mpuConnected = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println(F("✅ MPU6050 connected"));
    calibrateMPU();
  }

  // 5. Wi-Fi SoftAP (AP+STA mode for ESP-NOW compatibility)
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD, ESPNOW_CHANNEL);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("✅ SoftAP: SSID='%s'  IP=http://%s  Ch=%d\n",
                AP_SSID, ip.toString().c_str(), ESPNOW_CHANNEL);

  // 6. Web server
  server.on("/",             handleRoot);
  server.on("/api/telemetry",handleTelemetry);
  server.on("/api/cmd",      handleCommand);
  server.on("/api/settings", handleSettings);
  server.begin();
  Serial.println(F("✅ Web server running on port 80"));

  // 7. ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("❌ ESP-NOW init FAILED"));
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);

  memset(&camPeer, 0, sizeof(camPeer));
  memcpy(camPeer.peer_addr, camMAC, 6);
  camPeer.channel = ESPNOW_CHANNEL;
  camPeer.encrypt = false;
  if (esp_now_add_peer(&camPeer) == ESP_OK) {
    Serial.println(F("✅ ESP-NOW peer (ESP32-CAM) registered"));
  } else {
    Serial.println(F("⚠️  ESP-NOW peer add FAILED — check ESPCAM_MAC in secrets.h"));
  }

  lastSampleUs     = micros();
  lastManualCmdMs  = millis();

  // Broadcast initial mode (MANUAL) to CAM on start
  delay(500);   // Give ESP-NOW a moment to settle
  broadcastMode();

  addLog("🚀 System ready — MANUAL mode. Open http://" + ip.toString());
}

// ======================== MAIN LOOP ==========================================
unsigned long lastSensorMs = 0;

void loop() {
  // 1. High-frequency MPU gyro integration
  updateMPU();

  // 2. Web server client handling
  server.handleClient();

  // 3. Ultrasonic sensor at 100 ms intervals
  if (millis() - lastSensorMs >= 100) {
    lastSensorMs     = millis();
    latestDistanceCm = readUltrasonic();

    // Manual mode obstacle safety stop
    if (currentMode == MODE_MANUAL &&
        currentMotorState == "FORWARD" &&
        latestDistanceCm > 0 && latestDistanceCm < obstacleThresholdCm) {
      stopMotors();
      addLog("🚨 [MANUAL SAFETY] Obstacle at " + String(latestDistanceCm, 1) + " cm!");
    }
  }

  // 4. Manual deadman timeout (stop motors if no command for 2 s)
  if (currentMode == MODE_MANUAL &&
      currentMotorState != "STOPPED" &&
      millis() - lastManualCmdMs > MANUAL_TIMEOUT_MS) {
    stopMotors();
  }

  // 5. Legacy autonomous obstacle avoidance
  processLegacyAuto();

  // 6. CAM-guided vision navigation
  processCamGuided();
}
