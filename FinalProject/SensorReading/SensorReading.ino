#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"
#include <esp_now.h>
#include <WiFi.h>

Adafruit_AM2320 am2320 = Adafruit_AM2320();

// Pin Definitions
#define MOTION_SENSOR_PIN 16
#define SOUND_SENSOR_PIN 10
#define sensorPower 7
#define sensorPin 5

#define MOTION 0
#define TEMP 1
#define HUM 2
#define SOUND 3
#define FLOOD 4

// ESP-NOW Receiver's MAC address
uint8_t broadcastAddress[] = {0x24, 0xEC, 0x4A, 0x0E, 0xB6, 0xE0}; 
volatile bool buttonPressed = false;
volatile unsigned long lastInterruptTime = 0;
int message;

// Queue and Semaphore
QueueHandle_t sensorQueue;
SemaphoreHandle_t queueSemaphore;

TaskHandle_t Motion_Handle;
TaskHandle_t SerialMoniter_Handle;
TaskHandle_t Fire_Handle;
TaskHandle_t Flood_Handle;
TaskHandle_t Sound_Handle;

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
        

        vTaskDelay(pdMS_TO_TICKS(1000));  // Check every 500ms
    }
}

// AM2320 Temperature and Humidity Sensor Task (Producer)
void Task_FireDetection(void *pvParameters) {
    while (1) {
        
        if (xSemaphoreTake(queueSemaphore, portMAX_DELAY) == pdTRUE) {
            if (uxQueueSpacesAvailable(sensorQueue) > 1) {
              SensorData data1 = {TEMP, am2320.readTemperature()};
              xQueueSend(sensorQueue, &data1, portMAX_DELAY);
              SensorData data2 = {HUM, am2320.readHumidity()};
              xQueueSend(sensorQueue, &data2, portMAX_DELAY);
  
            }
            xSemaphoreGive(queueSemaphore);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Check every 500ms
    }
}

void Task_Sound(void *pvParameters) {
  pinMode(SOUND_SENSOR_PIN, INPUT);

  while (1) {
        int soundState = digitalRead(SOUND_SENSOR_PIN);
        SensorData data = {SOUND, soundState};
        if (xSemaphoreTake(queueSemaphore, portMAX_DELAY) == pdTRUE) {
          if (uxQueueSpacesAvailable(sensorQueue) > 0) {
              xQueueSend(sensorQueue, &data, portMAX_DELAY);
          }
          xSemaphoreGive(queueSemaphore);
      }
      vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
  }
}
// Flood Detection Task (Producer)
void Task_FloodDetection(void *pvParameters) {
    pinMode(sensorPower, OUTPUT);
    digitalWrite(sensorPower, LOW);  // Initially turn off sensor

    while (1) {
        digitalWrite(sensorPower, HIGH); // Turn sensor ON
        vTaskDelay(pdMS_TO_TICKS(10));   // Small delay before reading

        int waterLevel = analogRead(sensorPin);
        digitalWrite(sensorPower, LOW); // Turn sensor OFF after reading

        SensorData data = {FLOOD, waterLevel};

        if (xSemaphoreTake(queueSemaphore, portMAX_DELAY) == pdTRUE) {
            if (uxQueueSpacesAvailable(sensorQueue) > 0) {
                xQueueSend(sensorQueue, &data, portMAX_DELAY);
            }
            xSemaphoreGive(queueSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}


// Name: onDataSent
// Description: Callback function for ESP NOW that is called when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
 // Check if the delivery was successful and print the status
 Serial0.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

// Serial Monitor Display Task (Consumer)
void Task_SerialMonitor(void *pvParameters) {
  
    SensorData receivedData;
    while (1) {
        if (xSemaphoreTake(queueSemaphore, portMAX_DELAY) == pdTRUE) {
          if(uxQueueSpacesAvailable(sensorQueue) < 10){
            if (xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
                if(receivedData.sensor == MOTION && receivedData.data == 1){
                  Serial0.printf("Motion: %d\n", receivedData.data);
                  message = MOTION;
                  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&message, 32);
                } else if (receivedData.sensor == HUM && receivedData.data > 40){
                  Serial0.printf("Humidity: %d\n", receivedData.data);
                  message = HUM;
                  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&message, 32);
                } else if (receivedData.sensor == FLOOD && receivedData.data > 0){
                  Serial0.printf("Water Level: %d\n", receivedData.data);
                  message = FLOOD;
                  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&message, 32);
                } else if (receivedData.sensor == SOUND && receivedData.data == 1){
                  Serial0.printf("Sound: %d\n", receivedData.data);
                  message = SOUND;
                  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&message, 32);
                }
            }
          }
          xSemaphoreGive(queueSemaphore);
        }
    }
    
}

void setup() {
    Serial0.begin(115200);
    //while(!Serial);
    Serial0.println("starting");
    WiFi.mode(WIFI_STA);

    // Create Queue (size: 10 elements)
    sensorQueue = xQueueCreate(10, sizeof(SensorData));
    if (sensorQueue == NULL) {
        Serial0.println("Error creating queue!");
        while (1);
    }

    // Setup the ESP-NOW communication
    if (esp_now_init() != ESP_OK) return; // Initialize ESP-NOW and check for success
    esp_now_register_send_cb(onDataSent); // Register the send callback function

    esp_now_peer_info_t peerInfo; // Data structure for handling peer information
    // Copy the receiver's MAC address to peer information
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);  
    peerInfo.channel = 0; // Set WiFi channel to 0 (default)
    peerInfo.encrypt = false; // Disable encryption
    if (esp_now_add_peer(&peerInfo) != ESP_OK) return; // Add peer and check for success
    
    // Intitalize AM2320 temperature sensor
    Wire.begin(8,9);

    if (!am2320.begin()) {
    Serial0.println("Failed to detect AM2320 sensor! Check wiring.");
    while (1) { delay(10); } // Stop execution if sensor isn't found
    }
    Serial0.println("AM2320 sensor found!");
    am2320.begin();

    // Create Semaphore
    queueSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(queueSemaphore);  // Initialize semaphore

    // Create Producer Tasks (reading from sensor)
    xTaskCreatePinnedToCore(Task_Motion, "Motion Task", 4096, NULL, 1, &Motion_Handle, 0);
    xTaskCreatePinnedToCore(Task_FireDetection, "Fire Detection Task", 4096, NULL, 1, &Fire_Handle, 0);
    xTaskCreatePinnedToCore(Task_FloodDetection, "Flood Detection Task", 4096, NULL, 1, &Flood_Handle, 0);
    xTaskCreatePinnedToCore(Task_Sound, "Sound Task", 4096, NULL, 1, &Sound_Handle, 0);

    // Create Consumer Tasks (process sensor data)
    xTaskCreatePinnedToCore(Task_SerialMonitor, "Serial Task", 2048, NULL, 1, &SerialMoniter_Handle, 0);
}

void loop() {
    // FreeRTOS handles task scheduling
}
