/**
 * @file SensorReading.ino
 * @author Alison Tea
 * @author Shahnaz Mohideen
 * @date 22-March-2025
 * @brief Sensor monitoring system using FreeRTOS on ESP32
 * @details This program reads multiple sensors (motion, temperature, humidity, sound, and flood) 
 *          and sends the sensor data via ESP-NOW communication.
 */

// Libraries
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
#define MOTION_SENSOR_PIN 16  ///< PIR motion sensor pin
#define SOUND_SENSOR_PIN 10   ///< Sound sensor pin
#define sensorPower 7         ///< Flood sensor power pin
#define sensorPin 5           ///< Flood sensor data pin

// Sensor Types
#define MOTION 0  ///< Motion sensor type
#define TEMP 1    ///< Temperature sensor type
#define HUM 2     ///< Humidity sensor type
#define SOUND 3   ///< Sound sensor type
#define FLOOD 4   ///< Flood sensor type

// ESP-NOW Receiver's MAC address
uint8_t broadcastAddress[] = {0x24, 0xEC, 0x4A, 0x0E, 0xB6, 0xE0}; 
volatile bool buttonPressed = false;
volatile unsigned long lastInterruptTime = 0;
int message;

// Queue and Semaphore
QueueHandle_t sensorQueue;
SemaphoreHandle_t queueSemaphore;

// Task Handles
TaskHandle_t Motion_Handle;
TaskHandle_t SerialMoniter_Handle;
TaskHandle_t Fire_Handle;
TaskHandle_t Flood_Handle;
TaskHandle_t Sound_Handle;

/**
 * @struct SensorData
 * @brief Structure to hold sensor data
 */
typedef struct {
  int sensor;  ///< Sensor type identifier
  int data;    ///< Sensor reading value
} SensorData;

/**
 * @brief Task to monitor motion sensor
 * @param pvParameters Task parameters (unused)
 */
void Task_Motion(void *pvParameters) {
    pinMode(MOTION_SENSOR_PIN, INPUT_PULLUP);
    while (1) {
        int motionState = digitalRead(MOTION_SENSOR_PIN);
        SensorData data = {MOTION, motionState};

        if (xSemaphoreTake(queueSemaphore, portMAX_DELAY) == pdTRUE) {
            if (uxQueueSpacesAvailable(sensorQueue) > 0) {
                xQueueSend(sensorQueue, &data, portMAX_DELAY);
            }
            xSemaphoreGive(queueSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Task to monitor temperature and humidity using AM2320 sensor
 * @param pvParameters Task parameters (unused)
 */
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
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Task to monitor sound sensor
 * @param pvParameters Task parameters (unused)
 */
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
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Task to monitor flood detection sensor
 * @param pvParameters Task parameters (unused)
 */
void Task_FloodDetection(void *pvParameters) {
    pinMode(sensorPower, OUTPUT);
    digitalWrite(sensorPower, LOW);

    while (1) {
        digitalWrite(sensorPower, HIGH);
        vTaskDelay(pdMS_TO_TICKS(10));

        int waterLevel = analogRead(sensorPin);
        digitalWrite(sensorPower, LOW);

        SensorData data = {FLOOD, waterLevel};
        if (xSemaphoreTake(queueSemaphore, portMAX_DELAY) == pdTRUE) {
            if (uxQueueSpacesAvailable(sensorQueue) > 0) {
                xQueueSend(sensorQueue, &data, portMAX_DELAY);
            }
            xSemaphoreGive(queueSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief ESP-NOW data send callback
 * @param mac_addr Destination MAC address
 * @param status Status of the transmission
 */
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial0.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

/**
 * @brief Task to handle serial output
 * @param pvParameters Task parameters (unused)
 */
void Task_SerialMonitor(void *pvParameters) {
    SensorData receivedData;
    while (1) {
        if (xSemaphoreTake(queueSemaphore, portMAX_DELAY) == pdTRUE) {
            if (uxQueueSpacesAvailable(sensorQueue) < 10) {
                if (xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
                    Serial0.printf("Sensor: %d, Data: %d\n", receivedData.sensor, receivedData.data);
                    message = receivedData.sensor;
                    esp_now_send(broadcastAddress, (uint8_t *)&message, 32);
                }
            }
            xSemaphoreGive(queueSemaphore);
        }
    }
}

/**
 * @brief Arduino setup function
 */
void setup() {
    Serial0.begin(115200);
    WiFi.mode(WIFI_STA);
    sensorQueue = xQueueCreate(10, sizeof(SensorData));
    queueSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(queueSemaphore);

    esp_now_init();
    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    esp_now_add_peer(&peerInfo);

    Wire.begin(8, 9);
    am2320.begin();

    xTaskCreatePinnedToCore(Task_Motion, "Motion Task", 4096, NULL, 1, &Motion_Handle, 0);
    xTaskCreatePinnedToCore(Task_FireDetection, "Fire Task", 4096, NULL, 1, &Fire_Handle, 0);
    xTaskCreatePinnedToCore(Task_FloodDetection, "Flood Task", 4096, NULL, 1, &Flood_Handle, 0);
    xTaskCreatePinnedToCore(Task_Sound, "Sound Task", 4096, NULL, 1, &Sound_Handle, 0);
    xTaskCreatePinnedToCore(Task_SerialMonitor, "Serial Task", 2048, NULL, 1, &SerialMoniter_Handle, 0);
}

/**
 * @brief Arduino loop function (unused in FreeRTOS)
 */
void loop() {}
