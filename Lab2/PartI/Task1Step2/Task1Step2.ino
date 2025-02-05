// Filename: Task1Step2.ino
// Authors: Shahnaz Mohideen and Alison Tea
// Date: 2/4/2025
// Descripton: This sketch measures the time it takes for digitalWrite() to 
// change the output voltage to HIGH and then back to LOW.

// Includes
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_periph.h"

// Macros
#define GPIO_PIN 2  // GPIO Pin for LED

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial); // Wait for serial connection

  // Set pin as output
  pinMode(GPIO_PIN, OUTPUT);
}

void loop() {
  uint32_t totalTime = 0;

  // Measure the time for 1000 repetitions
  for (int i = 0; i < 1000; i++) {
    uint32_t startTime = micros();  // Start time
    digitalWrite(GPIO_PIN, HIGH);   // Set pin HIGH
    digitalWrite(GPIO_PIN, LOW);    // Set pin LOW
    uint32_t endTime = micros();    // End time

    totalTime += (endTime - startTime);  // Accumulate time difference
  }

  // Print the total time for 1000 repetitions
  Serial.print("Total time for 1000 repetitions: ");
  Serial.print(totalTime);
  Serial.println(" microseconds");

  delay(1000);
}
