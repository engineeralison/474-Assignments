#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Pin Definitions
#define MOTION_SENSOR_PIN 16

#define MOTION 1

// Queue and Semaphore
QueueHandle_t sensorQueue;
SemaphoreHandle_t queueSemaphore;

TaskHandle_t Motion_Handle;
TaskHandle_t SerialMoniter_Handle;

// Sensor Data Structure
typedef struct {
  int sensor;
  int data;
} SensorData;


// Ultrasonic Sensor Task (Producer)
void Task_Motion(void *pvParameters) {
    pinMode(MOTION_SENSOR_PIN, INPUT_PULLUP);  // Configure PIR sensor
    while (1) {
        int motionState = digitalRead(MOTION_SENSOR_PIN);
        SensorData data = {MOTION, motionState};

        if (xSemaphoreTake(queueSemaphore, portMAX_DELAY) == pdTRUE) {
            if (uxQueueSpacesAvailable(sensorQueue) > 0) {
                xQueueSend(sensorQueue, &data, portMAX_DELAY);
            }
            xSemaphoreGive(queueSemaphore);
        }
        

        vTaskDelay(pdMS_TO_TICKS(500));  // Check every 500ms
    }
}

// Serial Monitor Display Task (Consumer)
void Task_SerialMonitor(void *pvParameters) {
    SensorData receivedData;
    while (1) {
        if (xSemaphoreTake(queueSemaphore, portMAX_DELAY) == pdTRUE) {
          if(uxQueueSpacesAvailable(sensorQueue) < 10){
            if (xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
                if(receivedData.sensor == MOTION){
                Serial.printf("Motion: %d\n", receivedData.data);
                }
            }
          }
          xSemaphoreGive(queueSemaphore);
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("starting");

    // Create Queue (size: 10 elements)
    sensorQueue = xQueueCreate(10, sizeof(SensorData));
    if (sensorQueue == NULL) {
        Serial.println("Error creating queue!");
        while (1);
    }

    // Create Semaphore
    queueSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(queueSemaphore);  // Initialize semaphore

    // Create Producer Tasks (reading from sensor)
    xTaskCreatePinnedToCore(Task_Motion, "Motion Task", 2048, NULL, 1, &Motion_Handle, 0);

    // Create Consumer Tasks (process sensor data)
    xTaskCreatePinnedToCore(Task_SerialMonitor, "Serial Task", 2048, NULL, 1, &SerialMoniter_Handle, 0);
}

void loop() {
    // FreeRTOS handles task scheduling
}

