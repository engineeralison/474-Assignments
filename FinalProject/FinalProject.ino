#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "Wire.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"

Adafruit_AM2320 am2320 = Adafruit_AM2320();

// Pin Definitions
#define MOTION_SENSOR_PIN 16

#define MOTION 1

// Queue and Semaphore
QueueHandle_t sensorQueue;
SemaphoreHandle_t queueSemaphore;

TaskHandle_t Motion_Handle;
TaskHandle_t SerialMoniter_Handle;
TaskHandle_t TemperatureHumidity_Handle;

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


// AM2320 Temperature and Humidity Sensor Task (Producer)
void Task_TemperatureHumidity(void *pvParameters) {
    while (1) {
        
        if (xSemaphoreTake(queueSemaphore, portMAX_DELAY) == pdTRUE) {
            if (uxQueueSpacesAvailable(sensorQueue) > 0) {
              Serial.print("Temp: ");
              Serial.print(am2320.readTemperature());
              Serial.print(" C\t");

              Serial.print("Humidity: ");
              Serial.print(am2320.readHumidity());
              Serial.println(" %");
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
    while(!Serial);
    Serial.println("starting");

    // Create Queue (size: 10 elements)
    sensorQueue = xQueueCreate(10, sizeof(SensorData));
    if (sensorQueue == NULL) {
        Serial.println("Error creating queue!");
        while (1);
    }

    // Intitalize AM2320 temperature sensor
    Wire.begin(8,9);

    if (!am2320.begin()) {
    Serial.println("Failed to detect AM2320 sensor! Check wiring.");
    while (1) { delay(10); } // Stop execution if sensor isn't found
    }
    Serial.println("AM2320 sensor found!");
    am2320.begin();

    // Create Semaphore
    queueSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(queueSemaphore);  // Initialize semaphore

    // Create Producer Tasks (reading from sensor)
    xTaskCreatePinnedToCore(Task_Motion, "Motion Task", 2048, NULL, 1, &Motion_Handle, 0);
    xTaskCreatePinnedToCore(Task_TemperatureHumidity, "Temperature and Humidity Task", 2048, NULL, 1, &TemperatureHumidity_Handle, 0);
    // Create Consumer Tasks (process sensor data)
    xTaskCreatePinnedToCore(Task_SerialMonitor, "Serial Task", 2048, NULL, 1, &SerialMoniter_Handle, 0);
}

void loop() {
    // FreeRTOS handles task scheduling
}
