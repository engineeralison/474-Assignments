// FileName: Lab2ExtraCredit
// Authors: Alison Tea and Shahnaz Moidenn
// Date: 2/3/2025
// Descripton: This sketch prodcues a sequence of frequencies of the LED changing its brightness
// in a specific sequence and plays all the way through low -> higher -> highest.

// ====== Include's ========
#include <Arduino.h>
#include <driver/ledc.h>

// ======== Define's ==========
#define LED_PIN 5      // GPIO for LED
#define PR_PIN 1       // GPIO for Photoresistor
#define THRESHOLD 2000 // Adjust based on your environment

unsigned long lastSequenceTime = 0; // Timer tracking
const int sequenceDuration = 3000;  // Total sequence time (ms)
bool sequenceRunning = false;       // Prevents repeated triggering

void setup() {
    Serial.begin(115200);
    pinMode(PR_PIN, INPUT);

    // Attach LEDC to the LED pin (new API)
    ledcAttach(LED_PIN, 5000, 12);
}

// Name: playSequence
// Description: creates a sequence for the LED of the frequencies from low -> high -> highest
void playSequence() {
    Serial.println("Sequence Started!");

    // Brightness levels (low → high → highest)
    int levels[] = {512, 2048, 4095}; 
    int delays[] = {500, 500, 500};   // Duration per step

    for (int i = 0; i < 3; i++) {
        ledcWrite(LED_PIN, levels[i]);
        Serial.print("Brightness: ");
        Serial.println(levels[i]);
        delay(delays[i]);
    }

    ledcWrite(LED_PIN, 0); // Turn off LED after sequence
    Serial.println("Sequence Complete.");
}

void loop() {
    uint32_t lightValue = analogRead(PR_PIN);
    Serial.print("Light Level: ");
    Serial.println(lightValue);

    if (lightValue >= THRESHOLD && !sequenceRunning) {
        sequenceRunning = true;
        lastSequenceTime = millis();
        playSequence();
    }

    // Reset sequence flag after duration
    if (sequenceRunning && millis() - lastSequenceTime >= sequenceDuration) {
        sequenceRunning = false;
    }

    delay(100); // Short delay for stability
}
