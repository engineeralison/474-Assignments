// Includes
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// LED Pin Definitions
#define RED_LED    19
#define GREEN_LED  20
#define YELLOW_LED 21

// Traffic light states
typedef enum {
    RED,
    GREEN,
    YELLOW
} TrafficState_t;

// FreeRTOS Queue Handle
QueueHandle_t trafficQueue;

void setup() {
    Serial.begin(115200);

    // Configure LED pins
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);

    // Create a queue to hold one TrafficState_t value
    trafficQueue = xQueueCreate(1, sizeof(TrafficState_t));

    // Create tasks
    xTaskCreatePinnedToCore(Task_TrafficController, "TrafficController", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(Task_TrafficLight, "TrafficLight", 2048, NULL, 1, NULL, 0);
}

void loop() {
    // FreeRTOS handles task scheduling
}

// Traffic Controller Task (Producer) - Cycles through states and sends to queue
void Task_TrafficController(void *pvParameters) {
    TrafficState_t currentState = RED;

    while (1) {
        // Send current state to the queue
        xQueueSend(trafficQueue, &currentState, portMAX_DELAY);

        // Wait for 3 seconds before transitioning to the next state
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Cycle through RED -> GREEN -> YELLOW -> RED
        if (currentState == RED)
            currentState = GREEN;
        else if (currentState == GREEN)
            currentState = YELLOW;
        else
            currentState = RED;
    }
}

// Traffic Light Task (Consumer) - Receives state and updates LEDs
void Task_TrafficLight(void *pvParameters) {
    TrafficState_t receivedState;

    while (1) {
        // Wait for state from the queue
        if (xQueueReceive(trafficQueue, &receivedState, portMAX_DELAY)) {
            // Ensure only one LED is ON at a time
            digitalWrite(RED_LED, (receivedState == RED) ? HIGH : LOW);
            digitalWrite(GREEN_LED, (receivedState == GREEN) ? HIGH : LOW);
            digitalWrite(YELLOW_LED, (receivedState == YELLOW) ? HIGH : LOW);

            // Print the current state
            if (receivedState == RED)
                Serial.println("Traffic Light: RED (Stop)");
            else if (receivedState == GREEN)
                Serial.println("Traffic Light: GREEN (Go)");
            else
                Serial.println("Traffic Light: YELLOW (Slow Down)");
        }
    }
}


