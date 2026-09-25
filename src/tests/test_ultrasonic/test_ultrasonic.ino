/*
 * Ultrasonic Sensor (HC-SR04 / HC-SR04P) Test
 * Project: Auto Waste Bin
 * 
 * Pin Configuration:
 * - Trig : GPIO 4  (Safe digital output)
 * - Echo : GPIO 35 (Input-only pin on ESP32)
 * - VCC  : 5V (or 3.3V for HC-SR04P)
 * - GND  : Common GND
 * 
 * Electrical Note:
 * If using a 5V standard HC-SR04, the Echo pin outputs 5V. 
 * Since ESP32 GPIOs are 3.3V tolerant, it is recommended to use a simple 
 * resistor voltage divider (e.g. 1kΩ in series and 2kΩ to GND) or a 3.3V-compatible 
 * sensor (like HC-SR04P or RCWL-9610).
 */

#define TRIG_PIN 4
#define ECHO_PIN 35

// Speed of sound in air is ~343 m/s = 0.0343 cm/microsecond
const float SOUND_SPEED = 0.0343;

// Threshold for waste detection (e.g. user approaching bin or waste level)
const float DETECTION_THRESHOLD_CM = 20.0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Ensure trigger pin starts LOW
  digitalWrite(TRIG_PIN, LOW);

  Serial.println("\n=========================================");
  Serial.println("     ULTRASONIC SENSOR TEST BENCH        ");
  Serial.println("=========================================");
  Serial.printf("   Trig Pin : GPIO %d\n", TRIG_PIN);
  Serial.printf("   Echo Pin : GPIO %d (Input-Only)\n", ECHO_PIN);
  Serial.println("=========================================\n");
}

void loop() {
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond HIGH pulse to trigger ultrasonic burst
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure round-trip time of the echo pulse (timeout: 30ms ~ 5 meters max)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    Serial.println("⚠️  Out of range or no echo received! Check sensor wiring & power.");
  } else {
    // Calculate distance: (time * speed) / 2
    float distanceCm = (duration * SOUND_SPEED) / 2.0;
    float distanceInch = distanceCm / 2.54;

    Serial.printf("📏 Distance: %6.2f cm | %5.2f inches", distanceCm, distanceInch);

    // Visual indicator bar
    if (distanceCm <= DETECTION_THRESHOLD_CM) {
      Serial.println("  🚨 [OBJECT DETECTED NEAR BIN]");
    } else {
      Serial.println("  ✅ [CLEAR]");
    }
  }

  // Sample every 250ms
  delay(250);
}
