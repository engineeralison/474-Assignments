#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <LiquidCrystal_I2C.h>
#include <driver/ledc.h>

// Macros
#define PR_PIN 1
#define LED_PIN 5
#define SMA_WINDOW_SIZE 5

LiquidCrystal_I2C lcd(0x27, 16, 2);

SemaphoreHandle_t xLightSemaphore;
TaskHandle_t TaskHandle_LightDetector;
TaskHandle_t TaskHandle_LCD;

uint32_t lightReadings[SMA_WINDOW_SIZE] = {0};
uint8_t sma_index = 0;
uint32_t sum = 0;
uint32_t sma = 0;

void setup() {
  Serial0.begin(115200);
  Serial0.println("Starting");
  //while(!Serial);

  // setup lcd
  Wire.begin(8, 9);
  lcd.init();
  lcd.backlight();
  
  // setup photoresistor
  pinMode(PR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  xLightSemaphore = xSemaphoreCreateBinary();
  
  xSemaphoreGive(xLightSemaphore);
  if(xLightSemaphore != NULL){
    xTaskCreatePinnedToCore(Task_LightDetector, "LightDetector", 2048, NULL, 1, &TaskHandle_LightDetector, 0);
    xTaskCreatePinnedToCore(Task_LCD, "LCD", 2048, NULL, 1, &TaskHandle_LCD, 0);
  }
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
    Serial0.println("Starting PR task");
    if(xSemaphoreTake(xLightSemaphore, portMAX_DELAY) == pdTRUE){
      Serial0.println("Reading value");
      uint32_t value = analogRead(PR_PIN);
      
      // calculate SMA
      sum -= lightReadings[sma_index];
      lightReadings[sma_index] = value;
      sum += value;
      sma_index = (sma_index + 1) % SMA_WINDOW_SIZE;
      sma = sum / SMA_WINDOW_SIZE;

      // Signal other tasks
      xSemaphoreGive(xLightSemaphore);
    }

    // Debugging
    // Serial.print("Light Level: ");
    // Serial.print(value);
    // Serial.print(" | SMA: ");
    // Serial.println(sma);

    //vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms
    }
}

  


// ====================> TODO:
//          1. Initialize Variables
//          2. Loop Continuously
//           - Read light level from the photoresistor.
//           - Take semaphore
//           - Calculate the simple moving average and update variables.
//           - Give semaphore to signal data is ready.



void Task_LCD(void* args){
// ====================> TODO:
//          1. Initialize Variables
//           2. Loop Continuously
//            - Wait for semaphore.
//            - If data has changed, update the LCD with the new light level and SMA.
//            - Give back the semaphore.

  while(1){
    Serial0.println("Starting LCD task");
    if(xSemaphoreTake(xLightSemaphore, portMAX_DELAY) == pdTRUE){
      Serial0.println("Printing values");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Light: ");
      lcd.print(lightReadings[(sma_index + SMA_WINDOW_SIZE - 1) % SMA_WINDOW_SIZE]);
      lcd.setCursor(0, 1);
      lcd.print("SMA: ");
      lcd.print(sma);
      xSemaphoreGive(xLightSemaphore);
    }
  }
}
/*

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
*/
