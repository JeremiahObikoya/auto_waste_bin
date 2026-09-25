/*
 * L298N Dual H-Bridge Motor Driver Test
 * Project: Auto Waste Bin
 *
 * Pin Configuration:
 * - Motor A (Left Motor):
 *     ENA : GPIO 25 (Speed Control PWM)
 *     IN1 : GPIO 26 (Direction 1)
 *     IN2 : GPIO 27 (Direction 2)
 * - Motor B (Right Motor):
 *     ENB : GPIO 13 (Speed Control PWM)
 *     IN3 : GPIO 14 (Direction 1)
 *     IN4 : GPIO 19 (Direction 2)
 * - Power:
 *     L298N 12V/VCC : External Battery / Power Supply (e.g. 7.4V - 12V)
 *     L298N GND     : Common GND (Must connect ESP32 GND to Battery/Driver
 * GND!)
 *
 * Note:
 * Remove the ENA and ENB jumpers on the L298N board so the ESP32 can control
 * speed via PWM.
 */

// Motor A Pins (Left Motor)
#define PIN_ENA 25
#define PIN_IN1 26
#define PIN_IN2 27

// Motor B Pins (Right Motor)
#define PIN_ENB 13
#define PIN_IN3 14
#define PIN_IN4 19

// PWM Parameters for ESP32 Core
const int PWM_FREQ = 5000; // 5 kHz PWM frequency
const int PWM_RES = 8;     // 8-bit resolution (0 - 255 speed)

void setSpeedA(int speed) { analogWrite(PIN_ENA, constrain(speed, 0, 255)); }

void setSpeedB(int speed) { analogWrite(PIN_ENB, constrain(speed, 0, 255)); }

void stopMotors() {
  Serial.println("🛑 MOTORS: STOP");
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  setSpeedA(0);
  setSpeedB(0);
}

void moveForward(int speed) {
  Serial.printf("⬆️ MOTORS: FORWARD (Speed: %d/255)\n", speed);
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  setSpeedA(speed);
  setSpeedB(speed);
}

void moveBackward(int speed) {
  Serial.printf("⬇️ MOTORS: BACKWARD (Speed: %d/255)\n", speed);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  setSpeedA(speed);
  setSpeedB(speed);
}

void turnLeft(int speed) {
  Serial.printf("⬅️ MOTORS: TURN LEFT (Speed: %d/255)\n", speed);
  // Motor A reverse, Motor B forward
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  setSpeedA(speed);
  setSpeedB(speed);
}

void turnRight(int speed) {
  Serial.printf("➡️ MOTORS: TURN RIGHT (Speed: %d/255)\n", speed);
  // Motor A forward, Motor B reverse
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  setSpeedA(speed);
  setSpeedB(speed);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=========================================");
  Serial.println("       L298N MOTOR DRIVER TEST BENCH     ");
  Serial.println("=========================================");
  Serial.printf("   Motor A: ENA=%d, IN1=%d, IN2=%d\n", PIN_ENA, PIN_IN1,
                PIN_IN2);
  Serial.printf("   Motor B: ENB=%d, IN3=%d, IN4=%d\n", PIN_ENB, PIN_IN3,
                PIN_IN4);
  Serial.println("=========================================\n");

  // Configure direction pins as outputs
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Configure PWM speed pins as outputs
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  stopMotors();
  delay(2000);
}

void loop() {
  Serial.println("\n================ [ NEW TEST CYCLE ] ================");

  int mainSpeed = 110;
  int turnSpeed = 80;

  // 1. Move Forward
  moveForward(mainSpeed);
  delay(2000);

  // Stop
  stopMotors();
  delay(1000);

  // 2. Move Backward
  moveBackward(mainSpeed);
  delay(2000);

  // Stop
  stopMotors();
  delay(1000);

  // 3. Turn Left
  turnLeft(turnSpeed);
  delay(1500);

  // Stop
  stopMotors();
  delay(1000);

  // 4. Turn Right
  turnRight(turnSpeed);
  delay(1500);

  // Stop
  stopMotors();
  delay(1000);

  // 5. Speed Acceleration Ramp Test (Forward)
  Serial.println("📈 Ramping speed from 100 to 255...");
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  for (int s = 100; s <= 255; s += 25) {
    setSpeedA(s);
    setSpeedB(s);
    Serial.printf("   Current Speed: %d\n", s);
    delay(400);
  }

  // Final Stop
  stopMotors();
  Serial.println("⏸️ Cycle complete. Waiting 3 seconds before next cycle...\n");
  delay(3000);
}
