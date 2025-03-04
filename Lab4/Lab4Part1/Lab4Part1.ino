/**
 * @file Lab4_Part2.ino
 * @author Alison Tea
 * @author Shahnaz Mohideen
 * @date 5-March-2025
 * @brief ESP32 FreeRTOS Task Scheduler with LED, LCD Counter, and Alphabet Printer
 *
 * This program implements a real-time task scheduler using FreeRTOS on the ESP32.
 * It manages three tasks: blinking an LED, displaying a counter on an LCD,
 * and printing the alphabet to the serial monitor. A scheduler selects the task
 * with the shortest remaining execution time and runs it accordingly.
 */

// Libraries
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/ledc.h>
#include <LiquidCrystal_I2C.h>

// Macros
#define LED_PIN 5 ///< GPIO pin for LED

/// @brief LCD setup with I2C address 0x27, 16 columns, and 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

static int taskB_timer = 0; ///< Counter for the LCD task

/// @brief Task execution times in FreeRTOS ticks
const TickType_t ledTaskExecutionTime = 500 / portTICK_PERIOD_MS;      ///< LED blink task execution time (500ms)
const TickType_t counterTaskExecutionTime = 2000 / portTICK_PERIOD_MS; ///< Counter task execution time (2s)
const TickType_t alphabetTaskExecutionTime = 13000 / portTICK_PERIOD_MS; ///< Alphabet printing task execution time (13s)

/// @brief Remaining execution times for tasks
volatile TickType_t remainingLedTime = ledTaskExecutionTime;
volatile TickType_t remainingCounterTime = counterTaskExecutionTime;
volatile TickType_t remainingAlphabetTime = alphabetTaskExecutionTime;

/// @brief Task handles for FreeRTOS tasks
TaskHandle_t Task1_Handle = NULL;
TaskHandle_t Task2_Handle = NULL;
TaskHandle_t Task3_Handle = NULL;
TaskHandle_t Task4_Handle = NULL;

/**
 * @brief LED Blinking Task
 * 
 * This task blinks an LED at 1-second intervals and updates its remaining execution time.
 * @param arg Unused parameter
 */
void ledTask(void *arg) {
  while(1){
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    remainingLedTime -= (500 / portTICK_PERIOD_MS);
    if(remainingLedTime <= 0){
      remainingLedTime = ledTaskExecutionTime;
    }
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
    remainingLedTime -= (500 / portTICK_PERIOD_MS);
    if(remainingLedTime <= 0){
      remainingLedTime = ledTaskExecutionTime;
    }
  }
}

/**
 * @brief Counter Task
 * 
 * This task prints an incrementing counter to the LCD every second.
 * @param arg Unused parameter
 */
void counterTask(void *arg) {
  while(1){
    lcd.clear();
    for (taskB_timer = 1; taskB_timer <= 20; taskB_timer++) {
      lcd.setCursor(0, 0);
      lcd.print("Count: ");
      lcd.print(taskB_timer);
      vTaskDelay(pdMS_TO_TICKS(1000));
      remainingCounterTime -= (1000 / portTICK_PERIOD_MS);
      if(remainingCounterTime <= 0){
        remainingCounterTime = counterTaskExecutionTime;
      }
    }
  }
}

/**
 * @brief Alphabet Printing Task
 * 
 * This task prints the alphabet (A-Z) to the Serial Monitor, one letter per second.
 * @param arg Unused parameter
 */
void alphabetTask(void *arg) {
  while(1) {
    for (char letter = 'A'; letter <= 'Z'; letter++) {
      Serial0.p
