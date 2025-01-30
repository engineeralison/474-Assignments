#include <driver/ledc.h>

#define LED_PIN 5
#define PR_PIN 1

void setup() {
  ledcAttach(LED_PIN, 80000000, 12);

}

void loop() {
  int value = analogRead(PR_PIN);

  ledcWrite(0, value);
}
