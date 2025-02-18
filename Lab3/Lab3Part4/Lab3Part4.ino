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
#define NUM_TASKS 3
#define MAX_TASKS 5
#define BUTTON_PIN 20

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Global timers
static int taskA_timer = 0;
static int taskB_timer = 0;
static int taskC_timer = 0;
static int curr_task = 0;
volatile int end_task = false;
volatile unsigned long lastInterruptTime = 0;

// Function prototypes
void taskA();
void taskB();
void taskC();

// TCB struct
struct TCB {
 void (*taskFunction)();  // Pointer to the task function
 bool isRunning;          // State of the task
 bool isDone;             // Whether the task is completed or not
 int pid;                 // Unique process ID
};

// Array to store all the task control blocks (TCBs)
TCB TaskList [MAX_TASKS];

// function pointer typedef
typedef void (*funcPtr)();

funcPtr taskA_Ptr = taskA;
funcPtr taskB_Ptr = taskB;
funcPtr taskC_Ptr = taskC;


// Notes frequency array (example melody)
int melody[] = {262, 294, 330, 349, 392, 440, 494, 523, 587, 659}; 

void IRAM_ATTR buttonInterrupt() {
  unsigned long interruptTime = millis();
    // Debounce check: Ignore button presses that occur within 200ms of each other
    if (interruptTime - lastInterruptTime > 200) {  
        end_task = true;  // Set flag for processing in the main loop
        lastInterruptTime = interruptTime;
    }
}


void setup() {
    Serial0.begin(115200);
    Serial0.print("Starting\n");

    pinMode(LED_PIN, OUTPUT);

    Wire.begin(8,9);

    lcd.init();
    lcd.backlight();
    delay(2);
    
    ledcAttach(BUZZER_PIN, LEDC_FREQ, LEDC_RESOLUTION);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(BUTTON_PIN, buttonInterrupt, FALLING);

    // setup TCB list

    int j = 0;

    TaskList[j].taskFunction = taskA_Ptr;
    TaskList[j].isRunning = false;
    TaskList[j].isDone = true;
    TaskList[j].pid = j;
    j++;

    TaskList[j].taskFunction = taskB_Ptr;
    TaskList[j].isRunning = false;
    TaskList[j].isDone = true;
    TaskList[j].pid = j;
    j++;

    TaskList[j].taskFunction = taskC_Ptr;
    TaskList[j].isRunning = false;
    TaskList[j].isDone = true;
    TaskList[j].pid = j;
    j++;

    TaskList[j].taskFunction = NULL;

    
}


void loop() {
  
  TaskList[curr_task].isRunning = true;
  TaskList[curr_task].isDone = false;
  TaskList[curr_task].taskFunction();
  TaskList[curr_task].isRunning = false;
  TaskList[curr_task].isDone = true;
  curr_task = (curr_task + 1) % NUM_TASKS;

  delay(10);
  
}



// Description: Turns an external LED on and off eight times 
// in one-second intervals.
void taskA() {
   Serial0.print("LED Blinker\n");
   for(taskA_timer = 1; taskA_timer <= 8; taskA_timer++){
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
        taskA_timer++;

        if(end_task) {
          end_task = false;
          return;
        }
  }

}

// Description: Counts up from 1 to 10 on LCD.
void taskB() {
    Serial0.print("Counter\n");
    lcd.clear();
    for (taskB_timer = 1; taskB_timer <= 10; taskB_timer++) {
        lcd.setCursor(0, 0);
        lcd.print("Count: ");
        lcd.print(taskB_timer);
        delay(1000);

        if(end_task) {
          end_task = false;
          return;
        }
    }
}

// Description: Plays a melody and displays the note voltage levels.
void taskC() {
    Serial0.print("Music Player\n");
    ledcAttach(BUZZER_PIN, LEDC_FREQ, LEDC_RESOLUTION);

    for (taskC_timer = 0; taskC_timer < 10; taskC_timer++) {
        int freq = melody[taskC_timer];

        // Output frequency using LEDC
        ledcWrite(BUZZER_PIN, freq);
        Serial0.print("Playing note: ");
        Serial0.println(freq);

        delay(500);

        if(end_task) {
          end_task = false;
          return;
        }
    }
    ledcWrite(BUZZER_PIN, 0); // Stop tone

    
}
