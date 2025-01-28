#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_periph.h"

#define GPIO_PIN 19  // PIN 19

volatile uint32_t* status_register;
volatile uint32_t* output_register;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // set pin as output
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_PIN], PIN_FUNC_GPIO);

  status_register = (volatile uint32_t*) GPIO_ENABLE_REG;
  output_register = (volatile uint32_t*) GPIO_OUT_REG;
 
  (*status_register) |= (1<< GPIO_PIN); // sets pins 19 as an input/output
  (*output_register) &= ~(1<<GPIO_PIN);
}

void loop() {

  uint32_t total_time = 0;

  // for 1000 repetitions:
  // measure time to 
  // - turn pin's otupt to HIGH
  // - turn pin's output to LOW 
  // print out total time to serial monitor
  // 1 sec delay
  for(int i = 0; i<1000; i++){
    uint32_t start_time = micros();

    (*status_register) |= (1<< GPIO_PIN); 
    (*output_register) &= ~(1 << GPIO_PIN);

    uint32_t end_time = micros();
    total_time = end_time - start_time; 
    
  }

 Serial.println("Total time for 1000 repetitions: ");
 Serial.println(total_time);
 delay(1000);
  

}
