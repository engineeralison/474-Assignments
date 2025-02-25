// Includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pin Definitions
#define LED_FIXED 4
#define LED_VARIABLE 2
#define POT_PIN 5

// Task handles initalization
TaskHandle_t Task1_Handle = NULL;
TaskHandle_t Task2_Handle = NULL;

void setup() {
  Serial0.begin(115200);

  xTaskCreate(Task_FixedBlink, "FixedBlink", 1024, NULL, 1, &Task1_Handle);
  xTaskCreate(Task_VariableBlink, "VariableBlink", 1024, NULL, 1, &Task2_Handle);
}

void loop() {
  // FreeRTOS doesn't (usually) use loop()

}

// Task function for the Fixed Blinking LED
void Task_FixedBlink (void *pvParameter){
  pinMode(LED_FIXED, OUTPUT);

  while(1){
    digitalWrite(LED_FIXED, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_FIXED, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void Task_VariableBlink (void *pvParameter){
  pinMode(LED_VARIABLE, OUTPUT);
  pinMode(POT_PIN, INPUT);
  analogReadResolution(12); // sets ADC resolution to 12-bit (0-4095)

  while(1){
    int potValue = analogRead(POT_PIN);
    int delayTime = map(potValue, 0, 4095, 100, 1000);

    digitalWrite(LED_VARIABLE, HIGH);
    vTaskDelay(pdMS_TO_TICKS(delayTime));
    digitalWrite(LED_VARIABLE, LOW);
    vTaskDelay(pdMS_TO_TICKS(delayTime));
  }
}