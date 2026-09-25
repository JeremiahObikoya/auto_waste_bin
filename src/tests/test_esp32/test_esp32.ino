/*
 * ESP32 Built-in LED Blink Test
 * Project: Auto Waste Bin
 * 
 * Note:
 * - On most standard ESP32 boards (e.g. ESP32 Dev Module, DOIT DevKit V1, NodeMCU-32S),
 *   the built-in LED is connected to GPIO 2.
 */

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("=====================================");
  Serial.println("   ESP32 LED Blink Test Started      ");
  Serial.printf("   Target LED Pin: GPIO %d           \n", LED_BUILTIN);
  Serial.println("=====================================");
}

void loop() {
  // Turn LED ON
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("ESP32 LED: ON");
  delay(1000);

  // Turn LED OFF
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("ESP32 LED: OFF");
  delay(1000);
}
