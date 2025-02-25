#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Define LED pins
#define RED_LED 19
#define YELLOW_LED 20
#define GREEN_LED 21

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
            // Turn off all LEDs
            digitalWrite(RED_LED, LOW);
            digitalWrite(YELLOW_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            // Turn on the correct LED based on state
            switch (currentState) {
                case RED:
                    digitalWrite(RED_LED, HIGH);
                    Serial.println("RED Light ON");
                    break;
                case GREEN:
                    digitalWrite(GREEN_LED, HIGH);
                    Serial.println("GREEN Light ON");
                    break;
                case YELLOW:
                    digitalWrite(YELLOW_LED, HIGH);
                    Serial.println("YELLOW Light ON");
                    break;
            }
        }
    }
}

// Task to control the traffic light sequence
void Task_TrafficController(void *pvParameters) {
    TrafficLightState_t state = RED;
    while (1) {
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

    // Create tasks
    xTaskCreate(Task_TrafficController, "TrafficController", 1000, NULL, 1, NULL);
    xTaskCreate(Task_TrafficLight, "TrafficLight", 1000, NULL, 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();
}

void loop() {
    // Empty loop since FreeRTOS handles tasks
}
