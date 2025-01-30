#include <driver/ledc.h>

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
