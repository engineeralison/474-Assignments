/*
Filename: icte4part1.ino
Authors: Shahnaz Mohideen and Alison Tea
Date: 2/6/2025
Description: This sketch implements a round robin scheduler using function pointers. It runs 2 tasks, task A and
task B, which each blink an LED but at different rates. 
*/

// Includes
#include <Arduino.h>

// Macros
#define LED_PIN_1 5
#define LED_PIN_2 18

// global variable to keep track of time
static int taskA_timer = 0;
static int taskB_timer = 0;



// function prototypes
void taskA();
void taskB();
void executeTask(void (*functionPTR()));

// function pointer typedef
typedef void (*funcPtr)();

funcPtr taskA_Ptr = taskA;
funcPtr taskB_Ptr = taskB;

void setup() {
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
}

void loop() {
  executeTask(taskA_Ptr);
  executeTask(taskB_Ptr);
  delay(10);
}

// Name: executeTask
// Description: calls function given
void executeTask(funcPtr functionPTR){
  functionPTR();
}

// Name: taskA
// Description: blinks LED1 using counter
void taskA(){
  taskA_timer++;
  if(taskA_timer == 50){
    digitalWrite(LED_PIN_1, HIGH);
  }
  if(taskA_timer == 100){
    digitalWrite(LED_PIN_1, LOW);
    taskA_timer = 0;
  }
}

// Name: taskB
// Description: blinks LED2 using counter
void taskB(){
  taskB_timer++;
  if(taskB_timer == 100){
    digitalWrite(LED_PIN_2, HIGH);
  }
  if(taskB_timer == 200){
    digitalWrite(LED_PIN_2, LOW);
    taskB_timer = 0;
  }
}

