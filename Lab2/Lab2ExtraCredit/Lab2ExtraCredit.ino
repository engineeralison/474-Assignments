// FileName: Lab2ExtraCredit
// Authors: Alison Tea and Shahnaz Moidenn
// Date: 2/3/2025
// Descripton: This sketch prodcues a sequence of frequencies of the LED changing its brightness
// in a specific sequence and plays all the way through low -> higher -> highest.

// ====== Includes ========
#include <Arduino.h>
#include <driver/ledc.h>
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_periph.h"
#include "soc/timer_group_reg.h"

// ======== Macros ==========
#define LED_PIN 5      // GPIO for LED
#define PR_PIN 1       // GPIO for Photoresistor
#define THRESHOLD 2000 // Adjust based on your environment
#define TIMER_DIVIDER_VALUE 80 // make it 1 Mhz
#define TIMER_INCREMENT_MODE (1<<30) // set the timer to increment
#define TIMER_ENABLE (1<<31) // Enable timer bit
#define LED_TOGGLE_INTERVAL 500000 // LED toggle interval: 500 ms for a 1 Mhz clock

uint32_t lastSequenceTime = 0; // Timer tracking
const int sequenceDuration = 3000000;  // Total sequence time (ms)
bool sequenceRunning = false;       // Prevents repeated triggering

void setup() {
    Serial.begin(9600);
    //pinMode(PR_PIN, INPUT);

  // we need to config the value of the timer
  uint32_t timer_config = (TIMER_DIVIDER_VALUE << 13);

  // Add increment mode and enable mode
  timer_config |= TIMER_INCREMENT_MODE;
  timer_config |= TIMER_ENABLE;

  // write the timer _config that we have created into the register
  *((volatile uint32_t*) (TIMG_T0CONFIG_REG(0))) = timer_config;

  // Trigger an update to apply the config (T0UPDATE)
  *((volatile uint32_t*) (TIMG_T0UPDATE_REG(0))) = 1;


    // Attach LEDC to the LED pin (new API)
    ledcAttach(LED_PIN, 5000, 12);
}

// Name: playSequence
// Description: creates a sequence for the LED of the frequencies from low -> high -> highest
void playSequence() {
    Serial.println("Sequence Started!");

    // Brightness levels (low → high → highest)
    int levels[] = {512, 2048, 4095}; 
    uint32_t current_time = 0;
    uint32_t start_time = 0;

    for (int i = 0; i < 3; i++) {
        ledcWrite(LED_PIN, levels[i]);
        Serial.print("Brightness: ");
        Serial.println(levels[i]);

        // 500 ms delay between brightness levels
        *((volatile uint32_t*) (TIMG_T0UPDATE_REG(0))) = 1;
        start_time = *((volatile uint32_t*) (TIMG_T0LO_REG(0)));
        while(1){
          *((volatile uint32_t*) (TIMG_T0UPDATE_REG(0))) = 1;
          current_time = *((volatile uint32_t*) (TIMG_T0LO_REG(0)));
          if(current_time - start_time >= LED_TOGGLE_INTERVAL){
            break;
          }
        }
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
        *((volatile uint32_t*) (TIMG_T0UPDATE_REG(0))) = 1;
        lastSequenceTime = *((volatile uint32_t*) (TIMG_T0LO_REG(0)));
        playSequence();
    }

    *((volatile uint32_t*) (TIMG_T0UPDATE_REG(0))) = 1;
    uint32_t current_time = *((volatile uint32_t*) (TIMG_T0LO_REG(0)));

    // Reset sequence flag after duration
    if (sequenceRunning && current_time - lastSequenceTime >= sequenceDuration) {
        sequenceRunning = false;
    }

    // 100 ms delay for stability
    *((volatile uint32_t*) (TIMG_T0UPDATE_REG(0))) = 1;
    uint32_t start_time = *((volatile uint32_t*) (TIMG_T0LO_REG(0)));
    while(1){
      *((volatile uint32_t*) (TIMG_T0UPDATE_REG(0))) = 1;
      current_time = *((volatile uint32_t*) (TIMG_T0LO_REG(0)));
      if(current_time - start_time >= 100000){
        break;
      }
    }

}
