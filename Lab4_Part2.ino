#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <LiquidCrystal_I2C.h>
#include <driver/ledc.h>

// Macros
#define PR_PIN 1
#define BUZZER_PIN 20
#define LED_PIN 5
#define LEDC_CHANNEL 0
#define LEDC_FREQ 1000
#define LEDC_RESOLUTION 8
#define SMA_WINDOW_SIZE 5

LiquidCrystal_I2C lcd(0x27, 16, 2);

SemaphoreHandle_t xLightSemaphore;
TaskHandle_t TaskHandle_LightDetector;

uint32_t lightReadings[SMA_WINDOW_SIZE] = {0};
uint8_t index = 0;
uint32_t sum = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // setup lcd
  lcd.init();
  lcd.backlight();
  
  // setup photoresistor
  pinMode(PR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  xLightSemaphore = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(Task_LightDetector, "LightDetector", 2048, NULL, 1, &TaskHandle_LightDetector, 0);
  
}
// ====================> TODO:
//         1. Initialize pins, serial, LCD, etc
//         2. Create binary semaphore for synchronizing light level data.
//         3. Create Tasks
//          - Create the `Light Detector Task` and assign it to Core 0.
//          - Create `LCD Task` and assign it to Core 0.
//          - Create `Anomaly Alarm Task` and assign it to Core 1.
//          - Create `Prime Calculation Task` and assign it to Core 1.
void loop() {}


void Task_LightDetector (void *pvParameters) {
  pinMode(LED_PIN, OUTPUT);
  pinMode(PR_PIN, INPUT);

  while(1){
    uint32_t value = analogRead(PR_PIN);
    
    // calculate SMA
    sum -= lightReadings[index];
    lightReadings[index] = value;
    sum += value;
    index = (index + 1) % SMA_WINDOW_SIZE;
    uint32_t sma = sum / SMA_WINDOW_SIZE;

    // Signal other tasks
    xSemaphoreGive(xLightSemaphore);

    // Debugging
    Serial.print("Light Level: ");
    Serial.print(value);
    Serial.print(" | SMA: ");
    Serial.println(sma);

    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms
    }
}

  

}
// ====================> TODO:
//          1. Initialize Variables
//          2. Loop Continuously
//           - Read light level from the photoresistor.
//           - Take semaphore
//           - Calculate the simple moving average and update variables.
//           - Give semaphore to signal data is ready.



LCD Task (Core 0)
// ====================> TODO:
//          1. Initialize Variables
//           2. Loop Continuously
//            - Wait for semaphore.
//            - If data has changed, update the LCD with the new light level and SMA.
//            - Give back the semaphore.


Anomaly Alarm Task (Core 1)
// ====================> TODO:
//            1. Loop Continuously
//             - Wait for semaphore.
//             - Check if SMA indicates a light anomaly (outside thresholds).
//             - If anomaly detected, flash a LED signal.
//             - Give back the semaphore.


Prime Calculation Task (Core 1)
// ====================> TODO:
//            1. Loop from 2 to 5000
//             - Check if the current number is prime.
//             - If prime, print the number to the serial monitor
