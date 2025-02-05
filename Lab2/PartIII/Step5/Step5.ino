// Filename: Step5.ino
// Authors: Shahnaz Mohideen and Alison Tea
// Date: 2/4/2025
// Description: This sketch reads the ambient lighting in the room and
// adjusts the brightness of the LED using the LEDC library functions 
// and analogRead()

// Includes
#include <driver/ledc.h>

// Macros
#define LED_PIN 5
#define PR_PIN 1

void setup() {

  pinMode(LED_PIN, OUTPUT);
  pinMode(PR_PIN, INPUT);

  ledcAttach(LED_PIN, 5000, 12);

}

void loop() {
  uint32_t value = analogRead(PR_PIN);

  bool res = ledcWrite(LED_PIN, value);
}
