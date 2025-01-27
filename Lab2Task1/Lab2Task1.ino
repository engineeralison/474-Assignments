#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_periph.h"

// Define the GPIO pin number
#define GPIO_PIN 19  

// Pointers to GPIO registers
volatile uint32_t* status_register;
volatile uint32_t* output_register;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for the Serial connection
  Serial.println(sizeof(int));
  Serial.println("Starting ESP32 LED Blink");

  // Set pin 19 to GPIO function (configure it as a general-purpose input/output)
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_PIN], PIN_FUNC_GPIO);

  // Cast GPIO_ENABLE_REG and GPIO_OUT_REG macros to volatile pointers
  status_register = (volatile uint32_t*)GPIO_ENABLE_REG;  // GPIO enable register
  output_register = (volatile uint32_t*)GPIO_OUT_REG;     // GPIO output register

  // Set pin 19 as an output
  (*status_register) |= (1 << GPIO_PIN);  // Enable output mode for pin 19
  (*output_register) &= ~(1 << GPIO_PIN);  // Ensure the LED starts in the off state

  Serial.print("Initial status_register value: ");
  Serial.println(*status_register, HEX);
  Serial.print("Initial output_register value: ");
  Serial.println(*output_register, HEX);
}

void loop() {
 *(volatile uint32_t *)GPIO_OUT_REG |= (1 << GPIO_PIN);
  delay(1000);  // Wait for 1 second
 *(volatile uint32_t *)GPIO_OUT_REG &= ~(1 << GPIO_PIN);
 delay(1000);  // Wait for 1 second
}