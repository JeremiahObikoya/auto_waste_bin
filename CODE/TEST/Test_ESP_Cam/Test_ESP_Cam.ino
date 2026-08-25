/*
 * ESP32-CAM Built-in LED Blink Test (AI-Thinker Module)
 * Project: Auto Waste Bin
 * 
 * Note on ESP32-CAM on-board LEDs:
 * 1. Small Red Status LED (on the back of the board): Connected to GPIO 33.
 *    - Inverted Logic: LOW = ON, HIGH = OFF.
 * 2. Flashlight / Flash LED (bright white LED on the front): Connected to GPIO 4.
 *    - Standard Logic: HIGH = ON, LOW = OFF.
 */

#define STATUS_LED_PIN 33  // Small red status LED on the back (Active LOW)
#define FLASH_LED_PIN   4  // Bright flashlight LED on the front (Active HIGH)

// Set to true if you also want to blink the bright front flash LED
const bool ENABLE_FLASH_LED = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(STATUS_LED_PIN, OUTPUT);
  if (ENABLE_FLASH_LED) {
    pinMode(FLASH_LED_PIN, OUTPUT);
  }

  Serial.println("=========================================");
  Serial.println("   ESP32-CAM LED Blink Test Started      ");
  Serial.printf("   Status LED Pin: GPIO %d (Active LOW)  \n", STATUS_LED_PIN);
  if (ENABLE_FLASH_LED) {
    Serial.printf("   Flash LED Pin: GPIO %d (Active HIGH) \n", FLASH_LED_PIN);
  }
  Serial.println("=========================================");
}

void loop() {
  // Turn ON LED (LOW for GPIO 33)
  digitalWrite(STATUS_LED_PIN, LOW);
  if (ENABLE_FLASH_LED) {
    digitalWrite(FLASH_LED_PIN, HIGH);
  }
  Serial.println("ESP32-CAM LED: ON");
  delay(1000);

  // Turn OFF LED (HIGH for GPIO 33)
  digitalWrite(STATUS_LED_PIN, HIGH);
  if (ENABLE_FLASH_LED) {
    digitalWrite(FLASH_LED_PIN, LOW);
  }
  Serial.println("ESP32-CAM LED: OFF");
  delay(1000);
}
