#include "soc/timer_group_reg.h"

#define LED_PIN 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Starting");

  *(volatile uint32_t *)TIMG_T0CONFIG_REG(0) |= (1 << TIMG_T0_INCREASE);
  *(volatile uint32_t *)TIMG_T0CONFIG_REG(0) |= (1 << TIMG_T0_AUTORELOAD);
  *(volatile uint32_t *)TIMG_T0CONFIG_REG(0) |= (0x0050 << TIMG_T0_DIVIDER);
  *(volatile uint32_t *)TIMG_T0CONFIG_REG(0) |= (1 << TIMG_T0_EN);

}

void loop() {
  // put your main code here, to run repeatedly:
  *(volatile uint32_t *)TIMG_T0UPDATE_REG(0) |= (1 << 0);
  uint32_t timerValue = *(volatile uint32_t *)TIMG_T0LO_REG(0);
  Serial.println(timerValue);
  if(timerValue >= 0x0050){
    Serial.println("Reset");
  }
}
