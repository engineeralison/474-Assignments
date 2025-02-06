// Filename: Task1Step1.ino
// Authors: Alison Tea and Shahnaz Mohideen
// Date: 2/4/2025
// Description: This sketch uses pointers and registers to provide 
// direct control over hardware resources. 

// Includes:
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_periph.h"

// Macros:
#define GPIO_PIN 5 // Define the GPIO pin number

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for the Serial connection
  Serial.println("Starting ESP32 LED Blink");

  // Set pin 5 to GPIO function (configure it as a general-purpose input/output)
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_PIN], PIN_FUNC_GPIO);

  // Set pin 5 as an output
  *(volatile uint32_t *)GPIO_ENABLE_REG |= (1 << GPIO_PIN);  // Enable output mode for pin 19
 
  *(volatile uint32_t *)GPIO_OUT_REG &= ~(1 << GPIO_PIN);  // Ensure the LED starts in the off state

  Serial.print("Initial status_register value: ");
  Serial.println(*(volatile uint32_t *)GPIO_ENABLE_REG, HEX);
  Serial.print("Initial output_register value: ");
  Serial.println(*(volatile uint32_t *)GPIO_OUT_REG, HEX);
}

void loop() {
 *(volatile uint32_t *)GPIO_OUT_REG |= (1 << GPIO_PIN);
  delay(1000);  // Wait for 1 second
 *(volatile uint32_t *)GPIO_OUT_REG &= ~(1 << GPIO_PIN);
 delay(1000);  // Wait for 1 second
}
