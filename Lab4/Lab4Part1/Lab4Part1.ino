#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/ledc.h>
#include <LiquidCrystal_I2C.h>


#define LED_PIN 5

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

static int taskB_timer = 0;

// Total times for tasks
const TickType_t ledTaskExecutionTime = 500 / portTICK_PERIOD_MS;      // 500 ms
const TickType_t counterTaskExecutionTime = 2000 / portTICK_PERIOD_MS; // 2 seconds
const TickType_t alphabetTaskExecutionTime = 13000 / portTICK_PERIOD_MS; // 13 seconds

// Remaining Execution Times
volatile TickType_t remainingLedTime = ledTaskExecutionTime;
volatile TickType_t remainingCounterTime = counterTaskExecutionTime;
volatile TickType_t remainingAlphabetTime = alphabetTaskExecutionTime;

//Task handles initialization
TaskHandle_t Task1_Handle = NULL;
TaskHandle_t Task2_Handle = NULL;
TaskHandle_t Task3_Handle = NULL;
TaskHandle_t Task4_Handle = NULL;

// Name: ledTask
// Description: Turns an external LED on and off in one-second intervals.
void ledTask(void *arg) {
   // TODO: Blink an LED (1 second intervals), and update remaining time for this task
  while(1){
    //Serial0.println("LED Blinker");
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    remainingLedTime -= (500 / portTICK_PERIOD_MS);
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
    remainingLedTime -= (500 / portTICK_PERIOD_MS);
  }
}

// Name: counterTask
// Description: Counts up from 1 to 10 on LCD.
void counterTask(void *arg) {
 // TODO: Print out an incrementing counter to your LCD (1 second intervals), and 
 //       update remaining time for this task
  while(1){
    //Serial0.println("Counter");
    lcd.clear();
    for (taskB_timer = 1; taskB_timer <= 20; taskB_timer++) {
      lcd.setCursor(0, 0);
      lcd.print("Count: ");
      lcd.print(taskB_timer);
      vTaskDelay(pdMS_TO_TICKS(1000));
      remainingCounterTime -= (1000 / portTICK_PERIOD_MS);
    }
    
  }
}

// Name: alphabetTask
// Description: Prints A-Z to Serial Monitor.
void alphabetTask(void *arg) {
 // TODO: Print out the alphabet to Serial (1 second intervals), and update remaining
 //       time for this task

  while(1) {
    //Serial0.println("Alphabet Printer");
    for (char letter = 'A'; letter <= 'Z'; letter++) {
      Serial0.print(letter);
      Serial0.print(" ");
      vTaskDelay(pdMS_TO_TICKS(1000));
      remainingAlphabetTime -= (1000 / portTICK_PERIOD_MS);
    }
    Serial0.println();
  }
}


void scheduleTasks(void *arg) {
   // TODO: Implement SRTF scheduling logic. This function should select the task with 
   //       the shortest remaining time and run it. Once a task completes it should 
   //       reset its remaining time.
  
  while(1){
    TickType_t minTime = portMAX_DELAY;
    TaskHandle_t nextTask = NULL;

    
    if(remainingLedTime <= 0 && remainingCounterTime <= 0 && remainingAlphabetTime <= 0){
      remainingLedTime = ledTaskExecutionTime;
      remainingCounterTime = counterTaskExecutionTime;
      remainingAlphabetTime = alphabetTaskExecutionTime;
    }
    

    // Find the task with the shortest remaining time
    if(remainingLedTime <= 0){
      vTaskSuspend(Task1_Handle);
    }else if (remainingLedTime < minTime && remainingLedTime > 0) {
      minTime = remainingLedTime;
      nextTask = Task1_Handle;
    }

    if(remainingCounterTime <= 0){
      vTaskSuspend(Task2_Handle);
    }else if (remainingCounterTime < minTime && remainingCounterTime > 0) {
      minTime = remainingCounterTime;
      nextTask = Task2_Handle;
    }

    if(remainingAlphabetTime <= 0){
      vTaskSuspend(Task3_Handle);
    }else if (remainingAlphabetTime < minTime && remainingAlphabetTime > 0) {
      minTime = remainingAlphabetTime;
      nextTask = Task3_Handle;
    }

    // Resume the selected task and suspend scheduler for its duration
    if (nextTask != NULL) {
      vTaskResume(nextTask);
      vTaskDelay(minTime);
    }
  }
}


void setup() {
   // TODO: Create 4 tasks and pin them to core 0:
   //          1. A scheduler that handles the scheduling of the other three tasks
   //          2. Blink an LED
   //          3. Print a counter to the LCD
   //          4. Print the alphabet to Serial

  Serial0.begin(115200);
  Wire.begin(8,9);
  lcd.init();
  lcd.backlight();
  pinMode(LED_PIN, OUTPUT);

  xTaskCreatePinnedToCore(ledTask, "LEDBlink", 2048, NULL, 1, &Task1_Handle, 0);
  xTaskCreatePinnedToCore(counterTask, "Counter", 4096, NULL, 1, &Task2_Handle, 0);
  xTaskCreatePinnedToCore(alphabetTask, "Alphabet", 2048, NULL, 1, &Task3_Handle, 0);

  xTaskCreatePinnedToCore(scheduleTasks, "Scheduler", 2048, NULL, 2, &Task4_Handle, 0);
}
void loop() {}
