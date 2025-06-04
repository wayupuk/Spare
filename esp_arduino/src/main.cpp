#include <Arduino.h>  // Include the Arduino core functions

#define LED_PIN 2  // GPIO 2 is usually the onboard LED

void setup() {
  pinMode(LED_PIN, OUTPUT); // Set LED pin as output
}

void loop() {
  digitalWrite(LED_PIN, HIGH); // Turn the LED on
  delay(1000);                 // Wait for 1 second
  digitalWrite(LED_PIN, LOW);  // Turn the LED off
  delay(1000);                 // Wait for 1 second
}
