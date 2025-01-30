// File Name: icte3.ino
// Names: Alison Tea, Shahnaz Mohideen
// Date: 01/28/2025
// Descripton: This file controls a GPIO pin using direct register access and
// implements a single timing mechanism using delays

// ======== INCLUDE'S ============
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_periph.h"

// ======== MACRO'S ===============
#define GPIO_PIN 5 

void setup() {
  // intalize serial communication
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Starting ESP32");

  // set pin function to GPIO
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_PIN], PIN_FUNC_GPIO);
  // enable pin as an output
  *(volatile uint32_t *)GPIO_ENABLE_REG |= (1<<GPIO_PIN); 
  *(volatile uint32_t *)GPIO_OUT_REG &= ~(1<<GPIO_PIN); // starts LED in off state

  Serial.print("Initial enable register value");
  Serial.println(*(volatile uint32_t *)GPIO_ENABLE_REG, HEX);
  Serial.print("Intial output register value");
  Serial.println(*(volatile uint32_t *)GPIO_OUT_REG, HEX);
}

void loop() {
  // define a static counter variable
  static uint32_t counter = 0;
  static bool light = false;

  // increment counter in each iteration of the loop
  counter+=1;

  // view the number counter is on
  // Serial.println(counter); 

  // toggle GPIO pin state when the counter reaches a certain value
  if (counter == 200000) {
    if (light) {
      // if light is turned off
      *(volatile uint32_t *)GPIO_OUT_REG &= ~(1 << GPIO_PIN);
    } else {
      // if light is ON
        *(volatile uint32_t *)GPIO_OUT_REG |= (1 << GPIO_PIN);
    }
        light = !light;

        Serial.print("Output register value: ");
        Serial.println(*(volatile uint32_t *)GPIO_OUT_REG, HEX); 

        // reset the counter after toggling pin
        counter = 0;
  }
  
}
