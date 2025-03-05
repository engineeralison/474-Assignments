/**
 * @file Lab4Part2.ino
 * @author Alison Tea
 * @author Shahnaz Mohideen
 * @date 5-February-2025
 * @brief Demonstrates preemptive scheduling with FreeRTOS and dual-core processing.
 *
 * This Lab is divided into two parts to use scheduling techniques. The first part
 * implements a Shortest Remaining Time First Scheduling algorithm using FreeRTOS.
 * The three tasks implemented involve an LED blinker, counter, and alphabet printer.
 * The second part uses the dual-core architecture of the ESP32 to capture and process
 * real-time sensor data, with binary semaphores to synchronize tasks.
 */


// Libraries:
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <LiquidCrystal_I2C.h>
#include <driver/ledc.h>

// Macros:
#define PR_PIN 1 //< The pin connected to the photoresistor 
#define LED_PIN 5 //< The pin connected to the red LED
#define SMA_WINDOW_SIZE 5 //< the value assigned to the sma window size

LiquidCrystal_I2C lcd(0x27, 16, 2);

SemaphoreHandle_t xBinarySemaphore;
TaskHandle_t TaskHandle_LightDetector;
TaskHandle_t TaskHandle_LCD;
TaskHandle_t TaskHandle_AnomalyAlarm;
TaskHandle_t TaskHandle_PrimeCalculation;

// Global Variables 
uint32_t lightReadings[SMA_WINDOW_SIZE] = {0}; ///< Stores recent light readings
uint8_t sma_index = 0; ///< Current index in the SMA window
uint32_t sum = 0; ///< Sum of light readings for SMA calculation

/**
 * @brief Arduino setup function used for initialization.
 * @details This function configures GPIO pins, initializes serial communication, 
 *          creates the binary semaphore, and assigns FreeRTOS tasks to ESP32 cores.
 */
void setup() {
  // Initialize pins, serial monitor, LCD
  Serial.begin(9600);
  while(!Serial);

  Wire.begin(8,9);
  lcd.init();
  lcd.backlight();
  delay(2);

  // Pin configuration
  pinMode(PR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Create Binary Semaphore
  xBinarySemaphore = xSemaphoreCreateBinary();

  // Assign tasks to core 0 or core 1
  if (xBinarySemaphore != NULL){
    xSemaphoreGive(xBinarySemaphore); // allow first task to proceed
    xTaskCreatePinnedToCore(Task_LightDetector, "LightDetector", 2048, NULL, 1, &TaskHandle_LightDetector, 0);
    xTaskCreatePinnedToCore(Task_LCD, "LCD", 2048, NULL, 1, &TaskHandle_LCD, 0);
    xTaskCreatePinnedToCore(Task_AnomalyAlarm, "AnomalyAlarm", 2048, NULL, 1, &TaskHandle_AnomalyAlarm, 1);
    xTaskCreatePinnedToCore(Task_PrimeCalculation, "PrimeCalculation", 2048, NULL, 1, &TaskHandle_PrimeCalculation, 1);
  }
  
  
}

/**
 * @brief Standard Arduino loop function.
 * @note This function is intentionally empty because FreeRTOS does not use the `loop()` function.
 */ 
void loop() {}

/**
 * @brief Reads light level from the photoresistor and calculates SMA.
 * @param pvParameters FreeRTOS task parameters (not used).
 */

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

/**
 * @brief Displays the latest light level and SMA on an LCD.
 * @details Waits for the semaphore before accessing shared data, then updates the LCD.
 * @param pvParameters Pointer to task parameters (unused).
 */
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

/**
 * @brief Detects light anomalies and flashes an LED warning.
 * @details If the SMA value exceeds a threshold, the LED flashes three times.
 * @param pvParameters Pointer to task parameters (unused).
 */
void Task_AnomalyAlarm (void *pvParameters) {
  ledcAttach(LED_PIN, 5000, 12);
  
    while (1) {
    
            uint32_t sma_value = sum / SMA_WINDOW_SIZE;
                for (int i = 0; i < 3; i++) {
                    ledcWrite(LED_PIN, 4095);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    ledcWrite(LED_PIN, 0);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                }
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
  
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }


/**
 * @brief Determines whether a given number is prime.
 * @param num The integer to check.
 * @return `true` if the number is prime, `false` otherwise.
 */
bool isPrime(int num) {
    if (num < 2) return false;
    for (int i = 2; i * i <= num; i++) {
        if (num % i == 0) return false;
    }
    return true;
}

/**
 * @brief Continuously calculates prime numbers up to a limit.
 * @details Finds and prints prime numbers up to 5000 in the background.
 * @param pvParameters Pointer to task parameters (unused).
 */
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