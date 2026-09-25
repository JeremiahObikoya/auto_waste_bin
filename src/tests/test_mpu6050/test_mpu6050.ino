/*
 * MPU6050 Turn Angle (Yaw) & Rotation Detection Test
 * Project: Auto Waste Bin
 *
 * Description:
 * Measures the waste bin's turning angle (Yaw / Z-axis rotation) using the MPU6050 gyroscope.
 * - Calibrates gyro at startup to minimize drift
 * - Integrates angular velocity to track exact degrees turned
 * - Identifies turning direction (Left / Right)
 * - Detects 90° and 120° turn milestones
 * - Send 'r' over Serial Monitor anytime to re-zero the angle!
 *
 * Pin Configuration (ESP32):
 * - SDA : GPIO 21
 * - SCL : GPIO 22
 * - VCC : 3.3V or 5V
 * - GND : Common GND
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Target Angle Milestones (Degrees)
const float TARGET_ANGLE_1 = 90.0;
const float TARGET_ANGLE_2 = 120.0;
const float ANGLE_TOLERANCE = 2.5; // +/- tolerance for exact milestone detection

Adafruit_MPU6050 mpu;

// Gyro calibration offset & tracking variables
float gyroZ_offset = 0.0;
float currentYaw = 0.0; // Current angle in degrees (- is Right/CW or Left/CCW depending on mount)
unsigned long lastSampleTime = 0;
unsigned long lastPrintTime = 0;

// Milestone flags to prevent spamming notifications
bool reached90 = false;
bool reached120 = false;

// Function to calibrate Gyro Z axis while the bin is stationary
void calibrateGyroZ() {
  Serial.println("\n[!] Calibrating Gyroscope... KEEP THE BIN COMPLETELY STILL!");
  float sumZ = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumZ += g.gyro.z; // rad/s
    delay(4);
  }

  gyroZ_offset = sumZ / samples;
  Serial.print("✅ Calibration Complete! Gyro Z Offset: ");
  Serial.print(gyroZ_offset * (180.0 / PI), 4);
  Serial.println(" deg/s");
  Serial.println("===============================================================");
  Serial.println("Ready to track turns. Tip: Type 'r' in Serial Monitor to reset angle.");
  Serial.println("===============================================================\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=========================================");
  Serial.println("    MPU6050 BIN TURN ANGLE DETECTOR      ");
  Serial.println("=========================================");

  // Initialize I2C on ESP32 pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Initialize MPU6050
  while (!mpu.begin()) {
    Serial.println("❌ MPU6050 not detected! Check wiring.");
    Serial.println("Retrying in 2 seconds...");
    delay(2000);
  }

  Serial.println("✅ MPU6050 successfully connected!");

  // Configure sensor
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibrate Gyro
  calibrateGyroZ();

  lastSampleTime = micros();
  lastPrintTime = millis();
}

void loop() {
  // Check for reset command from Serial Monitor
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      currentYaw = 0.0;
      reached90 = false;
      reached120 = false;
      Serial.println("\n🔄 [RESET] Turn angle re-zeroed to 0.00°\n");
    }
  }

  // Calculate elapsed time (dt) in seconds
  unsigned long currentTime = micros();
  float dt = (currentTime - lastSampleTime) / 1000000.0;
  lastSampleTime = currentTime;

  // Read gyroscope data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convert raw gyro Z (rad/s) minus offset to degrees per second
  float gyroZ_deg_s = (g.gyro.z - gyroZ_offset) * (180.0 / PI);

  // Deadband filter to ignore tiny sensor noise when stationary
  if (abs(gyroZ_deg_s) > 0.6) {
    currentYaw += gyroZ_deg_s * dt;
  }

  float absAngle = abs(currentYaw);

  // Milestone Detection (90° and 120°)
  if (absAngle >= TARGET_ANGLE_1 && !reached90) {
    reached90 = true;
    Serial.printf("\n🎯 >>> [MILESTONE REACHED] Turned 90.0°! (Current: %.1f°) <<<\n\n", currentYaw);
  }

  if (absAngle >= TARGET_ANGLE_2 && !reached120) {
    reached120 = true;
    Serial.printf("\n🎯 >>> [MILESTONE REACHED] Turned 120.0°! (Current: %.1f°) <<<\n\n", currentYaw);
  }

  // Re-arm flags if turned back below milestones
  if (absAngle < (TARGET_ANGLE_1 - 5.0)) reached90 = false;
  if (absAngle < (TARGET_ANGLE_2 - 5.0)) reached120 = false;

  // Print status every 150ms
  if (millis() - lastPrintTime >= 150) {
    lastPrintTime = millis();

    // Determine turning state & direction
    String directionStr = "Stationary";
    if (gyroZ_deg_s > 2.0) {
      directionStr = "Turning Left  ↺";
    } else if (gyroZ_deg_s < -2.0) {
      directionStr = "Turning Right ↻";
    }

    // Determine status relative to 90° and 120°
    String statusStr = "";
    if (absAngle < TARGET_ANGLE_1 - ANGLE_TOLERANCE) {
      float remaining90 = TARGET_ANGLE_1 - absAngle;
      statusStr = "Heading to 90° (" + String(remaining90, 1) + "° left)";
    } else if (abs(absAngle - TARGET_ANGLE_1) <= ANGLE_TOLERANCE) {
      statusStr = "🎯 EXACTLY 90° TURN";
    } else if (absAngle < TARGET_ANGLE_2 - ANGLE_TOLERANCE) {
      float remaining120 = TARGET_ANGLE_2 - absAngle;
      statusStr = "Passed 90° -> Heading to 120° (" + String(remaining120, 1) + "° left)";
    } else if (abs(absAngle - TARGET_ANGLE_2) <= ANGLE_TOLERANCE) {
      statusStr = "🎯 EXACTLY 120° TURN";
    } else {
      statusStr = "Passed 120° (" + String(absAngle - TARGET_ANGLE_2, 1) + "° past)";
    }

    // Output formatted information to Serial Monitor
    Serial.printf("📐 Turned: %+7.2f° | Speed: %+6.1f°/s | Motion: %-15s | Status: %s\n",
                  currentYaw, gyroZ_deg_s, directionStr.c_str(), statusStr.c_str());
  }
}