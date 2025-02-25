// Includes
#include <Arduino.h>
#include <driver/ledc.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LED_PIN 5   // External LED pin
#define BUZZER_PIN 12 // Buzzer pin for music notes
#define LEDC_CHANNEL 0
#define LEDC_FREQ 1000
#define LEDC_RESOLUTION 8
#define NUM_TASKS 4
#define MAX_TASKS 5
// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Global timers
static int taskA_timer = 0;
static int taskB_timer = 0;
static int taskC_timer = 0;
static int taskD_timer = 0;

// Function prototypes
void taskA();
void taskB();
void taskC();
void taskD();

// TCB struct
struct TCB {
 void (*taskFunction)();  // Pointer to the task function
 bool isRunning;          // State of the task
 bool isDone;             // Whether the task is completed or not
 int pid;                 // Unique process ID
 int priority;	      // The priority level of the task
};

// Array to store all the task control blocks (TCBs)
TCB TaskList [MAX_TASKS];

// function pointer typedef
typedef void (*funcPtr)();

funcPtr taskA_Ptr = taskA;
funcPtr taskB_Ptr = taskB;
funcPtr taskC_Ptr = taskC;
funcPtr taskD_Ptr = taskD;


// Notes frequency array (example melody)
int melody[] = {262, 294, 330, 349, 392, 440, 494, 523, 587, 659}; 

void setup() {
    Serial0.begin(115200);
    //while(!Serial);

    pinMode(LED_PIN, OUTPUT);

    Wire.begin(8,9);

    lcd.init();
    lcd.backlight();
    delay(2);
    
    ledcAttach(BUZZER_PIN, LEDC_FREQ, LEDC_RESOLUTION);

    // setup TCB list

    int j = 0;

    TaskList[j].taskFunction = taskA_Ptr;
    TaskList[j].isRunning = false;
    TaskList[j].isDone = false;
    TaskList[j].pid = j;
    TaskList[j].priority = 4;
    j++;

    TaskList[j].taskFunction = taskB_Ptr;
    TaskList[j].isRunning = false;
    TaskList[j].isDone = false;
    TaskList[j].pid = j;
    TaskList[j].priority = 3;
    j++;

    TaskList[j].taskFunction = taskC_Ptr;
    TaskList[j].isRunning = false;
    TaskList[j].isDone = false;
    TaskList[j].pid = j;
    TaskList[j].priority = 2;
    j++;

    TaskList[j].taskFunction = taskD_Ptr;
    TaskList[j].isRunning = false;
    TaskList[j].isDone = false;
    TaskList[j].pid = j;
    TaskList[j].priority = 1;
    j++;

    TaskList[j].taskFunction = NULL;

    
}


void loop() {
  sortTasks();
  for(int i = 0; i < MAX_TASKS; i++){
    if(TaskList[i].taskFunction != NULL && TaskList[i].isRunning == false && TaskList[i].isDone == false){
      TaskList[i].isRunning = true;
      TaskList[i].taskFunction(); // this executes the task by calling the function using the function pointer
      TaskList[i].isRunning = false;
      TaskList[i].isDone = true;
      Serial0.print(TaskList[i].priority);
      Serial0.println();
    }else{
      break;
    }
  }

  delay(10);

  for(int i = 0; i < NUM_TASKS; i++){
    TaskList[i].priority = TaskList[i].priority % 4 + 1;
    TaskList[i].isDone = false;
  }
}

// Name: sortTasks
// Descripion: sort tasks by priority using bubble sort
void sortTasks() {
    for (int i = 0; i < NUM_TASKS - 1; i++) {
        for (int j = 0; j < NUM_TASKS - i - 1; j++) {
            if (TaskList[j].priority < TaskList[j + 1].priority) {
                TCB temp = TaskList[j];
                TaskList[j] = TaskList[j + 1];
                TaskList[j + 1] = temp;
            }
        }
    }
}

// Description: Turns an external LED on and off eight times 
// in one-second intervals.
void taskA() {
    
   for(taskA_timer = 1; taskA_timer <= 8; taskA_timer++){
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
        taskA_timer++;
  }

  Serial0.print("LED Blinker: ");
}

// Description: Counts up from 1 to 10 on LCD.
void taskB() {
    lcd.clear();
    for (taskB_timer = 1; taskB_timer <= 10; taskB_timer++) {
        lcd.setCursor(0, 0);
        lcd.print("Count: ");
        lcd.print(taskB_timer);
        delay(1000);
    }

    Serial0.print("Counter: ");
}

// Description: Plays a melody and displays the note voltage levels.
void taskC() {
    ledcAttach(BUZZER_PIN, LEDC_FREQ, LEDC_RESOLUTION);

    for (taskC_timer = 0; taskC_timer < 10; taskC_timer++) {
        int freq = melody[taskC_timer];

        // Output frequency using LEDC
        ledcWrite(BUZZER_PIN, freq);
        Serial0.print("Playing note: ");
        Serial0.println(freq);

        delay(500);
    }
    ledcWrite(BUZZER_PIN, 0); // Stop tone

    Serial0.print("Music Player: ");
}


// Description: Prints A-Z to Serial Monitor.
void taskD() {
    for (char letter = 'A'; letter <= 'Z'; letter++) {
        Serial0.print(letter);
        Serial0.print(" ");
        delay(250);
    }
    Serial0.println();
    Serial0.print("Alphabet Printer: ");
}
