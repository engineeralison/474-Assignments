/*
Filename: icte4part2.ino
Authors: Shahnaz Mohideen and Alison Tea
Date: 2/6/2025
Description: This sketch builds off the round robin scheduler in part1 but implements a Task Control Block. 
It runs two tasks, task A and task B, which each blink an LED but at different rates. 
*/

// Includes
#include <Arduino.h>

// Macros
// setting up the pins
#define LED_PIN_1 5
#define LED_PIN_2 18

// Task States
#define N_MAX_TASK 10
#define STATE_RUNNING 0
#define STATE_READY 1
#define STATE_WAITING 2
#define STATE_INACTIVE 3

// global variable to keep track of time
static int taskA_timer = 0;
static int taskB_timer = 0;

// function prototypes
void taskA();
void taskB();

// function pinter typedef
typedef void (*funcPtr)();

funcPtr taskA_Ptr = taskA;
funcPtr taskB_Ptr = taskB;

typedef struct TCBStruct{
  void (*fptr) (void); // function pointer to task we want to run
  unsigned short int state; // state of task
  unsigned int delay; // Delay counter for task scheduling
} TCBStruct;

// Array to store all the task control blocks (TCBs)
TCBStruct TaskList [N_MAX_TASK];



void setup() {
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);

  int j = 0;

  TaskList[j].fptr = taskA_Ptr;
  TaskList[j].state = STATE_READY;
  TaskList[j].delay = 0;
  j++;

  TaskList[j].fptr = taskB_Ptr;
  TaskList[j].state = STATE_READY;
  TaskList[j].delay = 0;
  j++;

  TaskList[j].fptr = NULL;

}

void loop() {
  
  for(int i = 0; i < N_MAX_TASK; i++){
    if(TaskList[i].fptr != NULL && TaskList[i].state == STATE_READY){
      TaskList[i].state = STATE_RUNNING;
      TaskList[i].fptr(); // this executes the task by calling the function using the function pointer
      TaskList[i].state = STATE_READY;
    }else{
      break;
    }
  }
  delay(10);
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
  if(taskB_timer == 25){
    digitalWrite(LED_PIN_2, HIGH);
  }
  if(taskB_timer == 50){
    digitalWrite(LED_PIN_2, LOW);
    taskB_timer = 0;
  }
}


