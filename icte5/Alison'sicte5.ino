#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Define LED pins
#define RED_LED 2
#define YELLOW_LED 3
#define GREEN_LED 4

// Define Traffic Light States
typedef enum {
    RED,
    GREEN,
    YELLOW
} TrafficLightState_t;

// Queue handle
QueueHandle_t trafficQueue;

// Task to control the traffic light
void Task_TrafficLight(void *pvParameters) {
    TrafficLightState_t currentState;
    while (1) {
        if (xQueueReceive(trafficQueue, &currentState, portMAX_DELAY) == pdPASS) {
            Serial.println("Received new traffic light state from queue");
            // Turn off all LEDs
            digitalWrite(RED_LED, LOW);
            digitalWrite(YELLOW_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            // Turn on the correct LED based on state
            switch (currentState) {
                case RED:
                    digitalWrite(RED_LED, HIGH);
                    Serial.println("RED Light ON - Stop");
                    break;
                case GREEN:
                    digitalWrite(GREEN_LED, HIGH);
                    Serial.println("GREEN Light ON - Go");
                    break;
                case YELLOW:
                    digitalWrite(YELLOW_LED, HIGH);
                    Serial.println("YELLOW Light ON - Slow Down");
                    break;
            }
        }
    }
}

// Task to control the traffic light sequence
void Task_TrafficController(void *pvParameters) {
    TrafficLightState_t state = RED;
    while (1) {
        Serial.println("Sending new traffic light state to queue");
        // Send the current state to the queue
        xQueueSend(trafficQueue, &state, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(3000)); // 3 seconds delay
        
        // Move to the next state
        if (state == RED) {
            state = GREEN;
        } else if (state == GREEN) {
            state = YELLOW;
        } else {
            state = RED;
        }
    }
}

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);
    Serial.println("Initializing Traffic Light System");

    // Configure LED pins as output
    pinMode(RED_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);

    // Create the queue
    trafficQueue = xQueueCreate(1, sizeof(TrafficLightState_t));
    if (trafficQueue == NULL) {
        Serial.println("Failed to create queue");
        while (1);
    }
    Serial.println("Queue created successfully");

    // Create tasks
    Serial.println("Creating tasks");
    xTaskCreate(Task_TrafficController, "TrafficController", 1000, NULL, 1, NULL);
    xTaskCreate(Task_TrafficLight, "TrafficLight", 1000, NULL, 1, NULL);

    // Start the scheduler
    Serial.println("Starting FreeRTOS Scheduler");
    vTaskStartScheduler();
}

void loop() {
    // Empty loop since FreeRTOS handles tasks
}
