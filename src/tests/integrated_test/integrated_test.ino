/*
 * =========================================================================================
 * ALL-IN-ONE INTEGRATED TEST BENCH (ESP32)
 * Project: Auto Waste Bin (Final Year Project)
 * =========================================================================================
 *
 * Integrated Components:
 * 1. DC Motors + L298N Driver (Left & Right differential drive)
 * 2. Dual Servo Motors (ESP32Servo - Lid opening / Sorting mechanism)
 * 3. HC-SR04 Ultrasonic Distance Sensor (Obstacle / Waste detection)
 * 4. MPU6050 6-DOF IMU (Real-time Yaw / Turn Angle & Angular Velocity)
 *
 * Pinout Summary (ESP32):
 * -----------------------------------------------------------------------------------------
 * Component        | Pin / Function       | ESP32 GPIO
 * -----------------------------------------------------------------------------------------
 * L298N Motor A    | ENA (PWM Speed)      | GPIO 25
 * (Left Motor)     | IN1 (Direction 1)    | GPIO 26
 *                  | IN2 (Direction 2)    | GPIO 27
 * -----------------------------------------------------------------------------------------
 * L298N Motor B    | ENB (PWM Speed)      | GPIO 13
 * (Right Motor)    | IN3 (Direction 1)    | GPIO 14
 *                  | IN4 (Direction 2)    | GPIO 19
 * -----------------------------------------------------------------------------------------
 * Servos           | Servo 1 (Lid / Flap) | GPIO 32
 *                  | Servo 2 (Sorter)     | GPIO 33
 * -----------------------------------------------------------------------------------------
 * Ultrasonic       | TRIG                 | GPIO 4
 * (HC-SR04)        | ECHO (Input only)    | GPIO 35
 * -----------------------------------------------------------------------------------------
 * MPU6050 IMU      | SDA                  | GPIO 21
 * (I2C)            | SCL                  | GPIO 22
 * =========================================================================================
 *
 * Power & Ground Notice:
 * - Always connect a Common Ground (GND) between ESP32, L298N battery, and
 * Servo power supply.
 * - Do NOT power Servos or L298N directly from the ESP32 3.3V pin. Use external
 * 5V/6V/battery.
 * =========================================================================================
 */

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
#define SERVO1_PIN 32
#define SERVO2_PIN 33

// Ultrasonic
#define TRIG_PIN 4
#define ECHO_PIN 35

// I2C Pins for MPU6050
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// ======================== OBJECTS & CONSTANTS ====================
Adafruit_MPU6050 mpu;
Servo servo1;
Servo servo2;

const float SOUND_SPEED = 0.0343; // cm/microsecond
const float OBSTACLE_THRESHOLD_CM = 20.0;

// Motor default speeds (0 - 255)
const int DEFAULT_DRIVE_SPEED = 110;
const int DEFAULT_TURN_SPEED = 80;

// ======================== GLOBAL VARIABLES =======================
// MPU6050 Tracking
float gyroZ_offset = 0.0;
float currentYaw = 0.0; // Accumulated turn angle in degrees
unsigned long lastSampleTimeMicros = 0;

// Sensor Readings & State
float latestDistanceCm = -1.0;
int currentServo1Pos = 0;
int currentServo2Pos = 0;
String currentMotorState = "STOPPED";

// Timing intervals
unsigned long lastSensorReadMillis = 0;
unsigned long lastTelemetryPrintMillis = 0;
unsigned long lastAutoSequenceMillis = 0;

// Auto sequence state machine
bool autoDemoMode = true; // Set to true to automatically cycle all components,
                          // or false for manual Serial commands
int autoStep = 0;

// ======================== MOTOR FUNCTIONS ========================
void setMotorSpeeds(int speedA, int speedB) {
  analogWrite(PIN_ENA, constrain(speedA, 0, 255));
  analogWrite(PIN_ENB, constrain(speedB, 0, 255));
}

void stopMotors() {
  currentMotorState = "STOPPED";
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(0, 0);
}

void moveBackward(int speed = DEFAULT_DRIVE_SPEED) {
  currentMotorState = "BACKWARDFORWARD";
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(speed, speed);
}

void moveForward(int speed = 234) {
  currentMotorState = "FORWARD";
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  setMotorSpeeds(speed, speed);
}

void turnLeft(int speed = DEFAULT_TURN_SPEED) {
  currentMotorState = "TURNING LEFT";
  // Left reverse, Right forward
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  setMotorSpeeds(speed, speed);
}

void turnRight(int speed = DEFAULT_TURN_SPEED) {
  currentMotorState = "TURNING RIGHT";
  // Left forward, Right reverse
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  setMotorSpeeds(speed, speed);
}

// ======================== SERVO FUNCTIONS ========================
void setServoPositions(int angle1, int angle2) {
  currentServo1Pos = constrain(angle1, 0, 180);
  currentServo2Pos = constrain(angle2, 0, 180);
  servo1.write(currentServo1Pos);
  servo2.write(currentServo2Pos);
}

// ======================== SENSOR READINGS ========================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m)
  if (duration == 0) {
    return -1.0; // Out of range / No echo
  }
  return (duration * SOUND_SPEED) / 2.0;
}

void updateMPU() {
  unsigned long now = micros();
  float dt = (now - lastSampleTimeMicros) / 1000000.0;
  lastSampleTimeMicros = now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convert raw gyro Z to deg/s minus calibrated offset
  float gyroZ_deg_s = (g.gyro.z - gyroZ_offset) * (180.0 / PI);

  // Deadband noise filter
  if (abs(gyroZ_deg_s) > 0.6) {
    currentYaw += gyroZ_deg_s * dt;
  }
}

void calibrateMPU() {
  Serial.println("[!] Calibrating MPU6050 Gyro... Keep the bin stationary!");
  float sumZ = 0.0;
  const int samples = 400;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumZ += g.gyro.z;
    delay(4);
  }

  gyroZ_offset = sumZ / samples;
  Serial.printf("✅ MPU6050 Calibrated! Offset: %.4f deg/s\n\n",
                gyroZ_offset * (180.0 / PI));
}

// ======================== SETUP ==================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(
      "\n============================================================");
  Serial.println(
      "       AUTO WASTE BIN - COMPLETE INTEGRATED TEST BENCH      ");
  Serial.println(
      "============================================================");

  // 1. Configure & Attach Servos FIRST (Allocate hardware timers for ESP32Servo)
  // This prevents ESP32 LEDC PWM/analogWrite from stealing servo timers!
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1.setPeriodHertz(50); // Standard 50Hz servo frequency
  servo2.setPeriodHertz(50);

  // Standard pulse width 500us - 2400us for SG90 / MG90S / MG995 / MG996R
  if (servo1.attach(SERVO1_PIN, 500, 2400)) {
    Serial.printf("✅ Servo 1 Attached on GPIO %d\n", SERVO1_PIN);
  } else {
    Serial.printf("❌ Servo 1 Attach Failed on GPIO %d!\n", SERVO1_PIN);
  }

  if (servo2.attach(SERVO2_PIN, 500, 2400)) {
    Serial.printf("✅ Servo 2 Attached on GPIO %d\n", SERVO2_PIN);
  } else {
    Serial.printf("❌ Servo 2 Attach Failed on GPIO %d!\n", SERVO2_PIN);
  }

  setServoPositions(0, 0); // Home both servos to 0 degrees
  delay(500);

  // 2. Configure Motor Pins & Initial Stop
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  stopMotors();
  Serial.println("✅ Motor Driver Pins Initialized");

  // 3. Configure Ultrasonic Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.println("✅ Ultrasonic Sensor Initialized (Trig=4, Echo=35)");

  // 4. Configure MPU6050 I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!mpu.begin()) {
    Serial.println("❌ ERROR: MPU6050 not detected on I2C (SDA=21, SCL=22)! "
                   "Check wiring.");
  } else {
    Serial.println("✅ MPU6050 Connected successfully!");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    calibrateMPU();
  }

  lastSampleTimeMicros = micros();
  lastSensorReadMillis = millis();
  lastTelemetryPrintMillis = millis();
  lastAutoSequenceMillis = millis();

  printHelpMenu();
}

// ======================== SERIAL COMMAND HANDLER =================
void sweepServosTest() {
  Serial.println("\n🔄 Starting Servo Sweep Test (0° -> 180° -> 0°)...");
  for (int pos = 0; pos <= 180; pos += 10) {
    setServoPositions(pos, pos);
    delay(40);
  }
  delay(300);
  for (int pos = 180; pos >= 0; pos -= 10) {
    setServoPositions(pos, pos);
    delay(40);
  }
  Serial.println("✅ Servo Sweep Test Finished.\n");
}

void printHelpMenu() {
  Serial.println(
      "------------------------------------------------------------");
  Serial.println(
      "🎮 SERIAL TEST CONTROLS (Send characters in Serial Monitor):");
  Serial.println("   [w] Forward          | [s] Backward");
  Serial.println("   [a] Turn Left        | [d] Turn Right");
  Serial.println("   [x] Stop Motors      | [r] Reset MPU Turn Angle to 0°");
  Serial.println("   [1] Servo 1 Open 90° | [2] Servo 2 Open 90°");
  Serial.println("   [o] Open Both (90°)  | [c] Close Both (0°)");
  Serial.println("   [9] Servo Sweep Test | [m] Toggle Auto Demo ON/OFF");
  Serial.println(
      "------------------------------------------------------------\n");
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
    case 'w':
    case 'W':
      autoDemoMode = false;
      moveForward();
      break;
    case 's':
    case 'S':
      autoDemoMode = false;
      moveBackward();
      break;
    case 'a':
    case 'A':
      autoDemoMode = false;
      turnLeft();
      break;
    case 'd':
    case 'D':
      autoDemoMode = false;
      turnRight();
      break;
    case 'x':
    case 'X':
      stopMotors();
      break;
    case '1':
      autoDemoMode = false;
      currentServo1Pos = 90;
      servo1.write(90);
      Serial.println("🔓 Servo 1 (Lid) -> 90°");
      break;
    case '2':
      autoDemoMode = false;
      currentServo2Pos = 90;
      servo2.write(90);
      Serial.println("🔓 Servo 2 (Sorter) -> 90°");
      break;
    case 'o':
    case 'O':
      autoDemoMode = false;
      setServoPositions(90, 90);
      Serial.println("🔓 Both Servos OPEN (90°)");
      break;
    case 'c':
    case 'C':
    case '0':
      autoDemoMode = false;
      setServoPositions(0, 0);
      Serial.println("🔒 Both Servos CLOSED (0°)");
      break;
    case '9':
      autoDemoMode = false;
      sweepServosTest();
      break;
    case 'r':
    case 'R':
      currentYaw = 0.0;
      Serial.println("🔄 MPU6050 Turn Angle re-zeroed to 0.00°");
      break;
    case 'm':
    case 'M':
      autoDemoMode = !autoDemoMode;
      autoStep = 0;
      lastAutoSequenceMillis = millis();
      Serial.printf("🤖 Auto Demonstration Mode: %s\n",
                    autoDemoMode ? "ENABLED" : "DISABLED");
      if (!autoDemoMode)
        stopMotors();
      break;
    case 'h':
    case 'H':
      printHelpMenu();
      break;
    }
  }
}

// ======================== AUTOMATED COMPONENT DEMO ===============
void runAutoDemo() {
  if (!autoDemoMode)
    return;

  unsigned long currentMillis = millis();

  // Cycle steps every 2.5 seconds
  if (currentMillis - lastAutoSequenceMillis >= 2500) {
    lastAutoSequenceMillis = currentMillis;

    switch (autoStep) {
    case 0:
      Serial.println("\n>>> [DEMO STEP 1] Moving Forward (Motors) & Monitoring "
                     "Distance <<<");
      moveForward(110);
      setServoPositions(0, 0);
      autoStep++;
      break;

    case 1:
      Serial.println(
          "\n>>> [DEMO STEP 2] Stopping Motors & Opening Bin Lid (Servos) <<<");
      stopMotors();
      setServoPositions(90, 90);
      autoStep++;
      break;

    case 2:
      Serial.println("\n>>> [DEMO STEP 3] Closing Bin Lid (Servos) <<<");
      setServoPositions(0, 0);
      autoStep++;
      break;

    case 3:
      Serial.println(
          "\n>>> [DEMO STEP 4] Turning Left (Checking MPU6050 Turn Angle) <<<");
      turnLeft(90);
      autoStep++;
      break;

    case 4:
      Serial.println("\n>>> [DEMO STEP 5] Turning Right (Checking MPU6050 Turn "
                     "Angle) <<<");
      turnRight(90);
      autoStep++;
      break;

    case 5:
      Serial.println(
          "\n>>> [DEMO STEP 6] Stopping Motors - Demo Cycle Complete! <<<");
      stopMotors();
      autoStep = 0; // Restart cycle
      break;
    }
  }
}

// ======================== MAIN LOOP ==============================
void loop() {
  // 1. High frequency MPU Gyro integration
  updateMPU();

  // 2. Handle interactive commands from user via Serial
  handleSerialCommands();

  // 3. Run automated demo if enabled
  runAutoDemo();

  // 4. Read Ultrasonic Sensor every 100ms
  if (millis() - lastSensorReadMillis >= 100) {
    lastSensorReadMillis = millis();
    latestDistanceCm = readUltrasonic();

    // Safety feature: auto-stop if moving forward and obstacle is too close (<
    // 15 cm)
    if (currentMotorState == "FORWARD" && latestDistanceCm > 0 &&
        latestDistanceCm < 15.0) {
      stopMotors();
      Serial.println(
          "🚨 [SAFETY STOP] Obstacle detected too close by Ultrasonic!");
    }
  }

  // 5. Print unified live telemetry every 250ms
  if (millis() - lastTelemetryPrintMillis >= 250) {
    lastTelemetryPrintMillis = millis();

    // Format Ultrasonic reading
    String distStr = (latestDistanceCm >= 0)
                         ? (String(latestDistanceCm, 1) + " cm")
                         : "Out of Range";

    // Format Turn Status
    String turnStatus = "";
    float absAngle = abs(currentYaw);
    if (abs(absAngle - 90.0) <= 3.0) {
      turnStatus = "[🎯 AT 90° TURN]";
    } else if (abs(absAngle - 120.0) <= 3.0) {
      turnStatus = "[🎯 AT 120° TURN]";
    } else if (absAngle >= 120.0) {
      turnStatus = "[Passed 120°]";
    } else if (absAngle >= 90.0) {
      turnStatus = "[Passed 90°]";
    } else {
      turnStatus = "[< 90°]";
    }

    // Unified Telemetry Line
    Serial.printf("📊 [US]: %-12s | [MPU Angle]: %+6.1f° %-17s | [Servos]: "
                  "S1=%d° S2=%d° | [Motors]: %s\n",
                  distStr.c_str(), currentYaw, turnStatus.c_str(),
                  currentServo1Pos, currentServo2Pos,
                  currentMotorState.c_str());
  }
}
