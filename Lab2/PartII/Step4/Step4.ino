// Filename:Step4.ino
// Authors: Alison Tea and Shahnaz Mohideen
// Date: 2/4/2025
// Description: This sketch uses ESP32 timers to control an LED by using 
// three key timer registers and direct register access. 

// Includes
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_periph.h"
#include "soc/timer_group_reg.h"

// Macros
#define GPIO_PIN 5

#define TIMER_DIVIDER_VALUE 80 // make it 1 Mhz
#define TIMER_INCREMENT_MODE (1<<30) // set the timer to increment
#define TIMER_ENABLE (1<<31) // Enable timer bit
#define LED_TOGGLE_INTERVAL 1000000 // LED toggle interval: 1 second for a 1 Mhz clock

static uint32_t last_toggle_time = 0;

void setup() {
  
  // configure gpio pin for output
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_PIN], PIN_FUNC_GPIO);
  *(volatile uint32_t *)GPIO_ENABLE_REG |= (1<<GPIO_PIN); 

  // we need to config the value of the timer
  uint32_t timer_config = (TIMER_DIVIDER_VALUE << 13);

  // Add increment mode and enable mode
  timer_config |= TIMER_INCREMENT_MODE;
  timer_config |= TIMER_ENABLE;

  // write the timer _config that we have created into the register
  *((volatile uint32_t*) (TIMG_T0CONFIG_REG(0))) = timer_config;

  // Trigger an update to apply the config (T0UPDATE)
  *((volatile uint32_t*) (TIMG_T0UPDATE_REG(0))) = 1;

}

void loop() {

  // user the timer register LO_REG that holds the lower 32 bits of the timer value 
  uint32_t current_time = *((volatile uint32_t*) (TIMG_T0LO_REG(0)));

  // run the led flashing
  if(current_time - last_toggle_time >= LED_TOGGLE_INTERVAL){
    *(volatile uint32_t *)GPIO_OUT_REG ^= (1 << GPIO_PIN);  
    // store current time to ensure the LED toggles at a regular interval
    last_toggle_time = current_time;
  }

   // Trigger an update to apply the config (T0UPDATE)
  *((volatile uint32_t*) (TIMG_T0UPDATE_REG(0))) = 1;

}
