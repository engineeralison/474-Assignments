//Libraries
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RED 0
#define YELLOW 1
#define GREEN 2

QueueHandle_t myQueue;
int value;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  myQueue = xQueueCreate(3, sizeof(int)); 
  value = RED;
}

void loop() {
  xQueueSend(myQueue, &value, portMAX_DELAY);
  value = (value + 1) % 3;
  vTaskDelay(pdMS_TO_TICKS);
}



