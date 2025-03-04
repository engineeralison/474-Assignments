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

SemaphoreHandle_t xBinarySemaphore;
TaskHandle_t TaskHandle_LightDetector;
TaskHandle_t TaskHandle_LCD;
TaskHandle_t TaskHandle_AnomalyAlarm;
TaskHandle_t TaskHandle_PrimeCalculation;

uint32_t lightReadings[SMA_WINDOW_SIZE] = {0};
uint8_t sma_index = 0;
uint32_t sum = 0;

void setup() {
  Serial.begin(9600);
  while(!Serial);

  // setup lcd
  Wire.begin(8,9);
  lcd.init();
  lcd.backlight();
  delay(2);

  // setup photoresistor
  pinMode(PR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  xBinarySemaphore = xSemaphoreCreateBinary();

  if (xBinarySemaphore != NULL){
    xSemaphoreGive(xBinarySemaphore); // allow first task to proceed
    xTaskCreatePinnedToCore(Task_LightDetector, "LightDetector", 2048, NULL, 1, &TaskHandle_LightDetector, 0);
    xTaskCreatePinnedToCore(Task_LCD, "LCD", 2048, NULL, 1, &TaskHandle_LCD, 0);
    xTaskCreatePinnedToCore(Task_AnomalyAlarm, "AnomalyAlarm", 2048, NULL, 1, &TaskHandle_AnomalyAlarm, 1);
    xTaskCreatePinnedToCore(Task_PrimeCalculation, "PrimeCalculation", 2048, NULL, 1, &TaskHandle_PrimeCalculation, 1);
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
  Serial.println("Task_LightDetector is running...");

  pinMode(LED_PIN, OUTPUT);
  pinMode(PR_PIN, INPUT);

   while (1) {
        uint32_t value = analogRead(PR_PIN);
        //Serial.println("Waiting for semaphore...");
        //Serial.println(value);
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            // Calculate SMA
            sum -= lightReadings[sma_index];
            lightReadings[sma_index] = value;
            sum += lightReadings[sma_index];

            // Update index
            sma_index = (sma_index + 1) % SMA_WINDOW_SIZE;

            uint32_t sma_value = sum / SMA_WINDOW_SIZE;
            // Release semaphore
            xSemaphoreGive(xBinarySemaphore);
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for stability
    }
}

  
// ====================> TODO:
//          1. Initialize Variables
//          2. Loop Continuously
//           - Read light level from the photoresistor.
//           - Take semaphore
//           - Calculate the simple moving average and update variables.
//           - Give semaphore to signal data is ready.



void Task_LCD (void *pvParameters){
   char buffer[16];

    while (1) {
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            lcd.clear();
            lcd.setCursor(0, 0);
            sprintf(buffer, "Light: %lu", lightReadings[(sma_index + SMA_WINDOW_SIZE - 1) % SMA_WINDOW_SIZE]);
            lcd.print(buffer);
            lcd.setCursor(0, 1);
            sprintf(buffer, "SMA: %lu", sum / SMA_WINDOW_SIZE);
            lcd.print(buffer);
            xSemaphoreGive(xBinarySemaphore);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


// ====================> TODO:
//          1. Initialize Variables
//           2. Loop Continuously
//            - Wait for semaphore.
//            - If data has changed, update the LCD with the new light level and SMA.
//            - Give back the semaphore.


void Task_AnomalyAlarm (void *pvParameters) {
  //pinMode(LED_PIN, OUTPUT);
  ledcAttach(LED_PIN, 5000, 12);

    while (1) {
        //if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            uint32_t sma_value = sum / SMA_WINDOW_SIZE;

            if (sma_value > 3800 || sma_value < 300) {
                for (int i = 0; i < 3; i++) {
                    ledcWrite(LED_PIN, 4095);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    ledcWrite(LED_PIN, 0);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                }
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
            //xSemaphoreGive(xBinarySemaphore);
        //}
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// ====================> TODO:
//            1. Loop Continuously
//             - Wait for semaphore.
//             - Check if SMA indicates a light anomaly (outside thresholds).
//             - If anomaly detected, flash a LED signal.
//             - Give back the semaphore.
bool isPrime(int num) {
    if (num < 2) return false;
    for (int i = 2; i * i <= num; i++) {
        if (num % i == 0) return false;
    }
    return true;
}

void Task_PrimeCalculation (void *pvParameters) {
    int num = 2;
    while (1) {
        if (isPrime(num)) {
            Serial.println(num);
        }
        num++;
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Prevent CPU starvation
    }
}

// ====================> TODO:
//            1. Loop from 2 to 5000
//             - Check if the current number is prime.
//             - If prime, print the number to the serial monitor

