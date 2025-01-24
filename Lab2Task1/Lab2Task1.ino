#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_periph.h"

#define GPIO_PIN 19  // PIN 19

volatile uint32_t* status_register;
volatile uint32_t* output_register;

void setup() {

  // Setting pin 19 to be a general input/output pin
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_PIN], PIN_FUNC_GPIO);

  // Mark pin 19 as output using the GPIO_ENABLE_REG macro:
  // Casts GPIO_ENABLE_REG to a volatile pointer to a 32 bit unsigned int
  status_register = (volatile uint32_t*) GPIO_ENABLE_REG;
  output_register = (volatile uint32_t*) GPIO_OUT_REG;
 
  (*status_register) |= (1<< GPIO_PIN); // sets pins 19 as an input/output
  (*output_register) &= ~(1<<GPIO_PIN);
}

void loop() {
  // turn LED on (set corresponding bit in GPIO output reg)
  (*status_register) |= (1<< GPIO_PIN); 
  // Mark pin 19 as high output using the GPIO_OUT_REG macro
  // wait 1 second
  delay(1000);
  // Turn LED off (clear corresponding bit in GPIO output reg)
  // Mark Pin 19 as LOW ouptut using the GPIO_OUT_REG macro
  (*output_register) &= ~(1 << GPIO_PIN);
  // wait 1 second
  delay(1000);
}
