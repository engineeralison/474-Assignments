// Filename: Task1Step3.ino
// Authors: Alison Tea and Shahnaz Mohideen
// Date: 2/4/2025
// Descripition: This sketch compares the speed of arduino library 
// functions versus direct register access. 

// Includes
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_periph.h"

// Macros
#define GPIO_PIN 5 // Define the GPIO pin number

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial);  // Wait for the Serial connection
  Serial.println("Starting ESP32 LED Blink");

  // Set pin 2 to GPIO function (configure it as a general-purpose input/output)
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_PIN], PIN_FUNC_GPIO);

  // Set pin 2 as an output
  *(volatile uint32_t *)GPIO_ENABLE_REG |= (1 << GPIO_PIN);  // Enable output mode for pin
  *(volatile uint32_t *)GPIO_OUT_REG &= ~(1 << GPIO_PIN);  // Ensure the LED starts in the off state

  Serial.print("Initial status_register value: ");
  Serial.println(*(volatile uint32_t *)GPIO_ENABLE_REG, HEX);
  Serial.print("Initial output_register value: ");
  Serial.println(*(volatile uint32_t *)GPIO_OUT_REG, HEX);
}

void loop() {
  uint32_t startTime, endTime;
  uint32_t totalTimeDigitalWrite = 0, totalTimeDirectAccess = 0;

  // Measure time for 1000 repetitions using digitalWrite (Library function)
  for (int i = 0; i < 1000; i++) {
    startTime = micros();  // Start time for digitalWrite

    digitalWrite(GPIO_PIN, HIGH);  // Set pin HIGH using digitalWrite
    digitalWrite(GPIO_PIN, LOW);   // Set pin LOW using digitalWrite

    endTime = micros();  // End time for digitalWrite
    totalTimeDigitalWrite += (endTime - startTime);  // Accumulate time for digitalWrite
  }

  // Measure time for 1000 repetitions using direct register access
  for (int i = 0; i < 1000; i++) {
    startTime = micros();  // Start time for direct register access

    *(volatile uint32_t *)GPIO_OUT_REG |= (1 << GPIO_PIN);  // Set pin HIGH using direct register access
    *(volatile uint32_t *)GPIO_OUT_REG &= ~(1 << GPIO_PIN); // Set pin LOW using direct register access

    endTime = micros();  // End time for direct register access
    totalTimeDirectAccess += (endTime - startTime);  // Accumulate time for direct register access
  }

  // Print the total time for both methods
  Serial.print("Total time for 1000 repetitions using digitalWrite: ");
  Serial.print(totalTimeDigitalWrite);
  Serial.println(" microseconds");

  Serial.print("Total time for 1000 repetitions using direct register access: ");
  Serial.print(totalTimeDirectAccess);
  Serial.println(" microseconds");

  delay(1000);
}
