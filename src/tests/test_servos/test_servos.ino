/*
 * Dual Servo Motors Test
 * Project: Auto Waste Bin
 *
 * Pin Configuration:
 * - Servo 1 Signal : GPIO 32
 * - Servo 2 Signal : GPIO 33
 * - Servo VCC      : External 5V / 6V Power Supply (Do NOT power servos
 * directly from ESP32 3.3V pin!)
 * - Servo GND      : Common GND (Connect ESP32 GND to External Power Supply
 * GND)
 */

#include <ESP32Servo.h> // Changed from standard Servo.h

#define SERVO1_PIN 32
#define SERVO2_PIN 33

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=========================================");
  Serial.println("       DUAL SERVO MOTORS TEST BENCH      ");
  Serial.println("=========================================");
  Serial.printf("   Servo 1 Signal : GPIO %d\n", SERVO1_PIN);
  Serial.printf("   Servo 2 Signal : GPIO %d\n", SERVO2_PIN);
  Serial.println("=========================================\n");

  // Attach servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // Initial homing position (0 degrees)
  Serial.println("🔄 Homing both servos to 0°...");
  servo1.write(0);
  servo2.write(0);
  delay(1500);
}

void sweepServo(Servo &s, int servoNum, const char *name) {
  Serial.printf("➡️ Sweeping Servo %d (%s) from 0° -> 180°...\n", servoNum,
                name);
  for (int pos = 0; pos <= 180; pos += 5) {
    s.write(pos);
    delay(20);
  }
  delay(500);

  Serial.printf("⬅️ Sweeping Servo %d (%s) from 180° -> 0°...\n", servoNum,
                name);
  for (int pos = 180; pos >= 0; pos -= 5) {
    s.write(pos);
    delay(20);
  }
  delay(500);
}

void loop() {
  Serial.println("\n--- [TEST 1: Individual Sweep - Servo 1] ---");
  sweepServo(servo1, 1, "Bin Lid / Flap 1");
  delay(1000);

  Serial.println("\n--- [TEST 2: Individual Sweep - Servo 2] ---");
  sweepServo(servo2, 2, "Sorter / Flap 2");
  delay(1000);

  Serial.println("\n--- [TEST 3: Simultaneous Open & Close Action] ---");

  // Step 1: Open / Deploy position (90 degrees)
  Serial.println("🔓 Opening Lid / Mechanism to 90°...");
  servo1.write(90);
  servo2.write(90);
  delay(2000);

  // Step 2: Full Extension (180 degrees)
  Serial.println("🔄 Full Extension to 180°...");
  servo1.write(180);
  servo2.write(180);
  delay(2000);

  // Step 3: Close / Return to Home (0 degrees)
  Serial.println("🔒 Returning to Home Position 0°...");
  servo1.write(0);
  servo2.write(0);
  delay(2000);
}