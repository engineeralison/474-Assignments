// Includes
#include <driver/ledc.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LED_PIN 5   // External LED pin
#define BUZZER_PIN 12 // Buzzer pin for music notes
#define LEDC_CHANNEL 0
#define LEDC_FREQ 1000
#define LEDC_RESOLUTION 8

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Global timers
static int taskA_timer = 0;
static int taskB_timer = 0;
static int taskC_timer = 0;
static int taskD_timer = 0;

// Notes frequency array (example melody)
int melody[] = {262, 294, 330, 349, 392, 440, 494, 523, 587, 659}; 

// Description: Turns an external LED on and off eight times 
// in one-second intervals.
void taskA() {
    if (taskA_timer < 8) {
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
        taskA_timer++;
    }
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
}

// Description: Plays a melody and displays the note voltage levels.
void taskC() {
    ledcAttach(BUZZER_PIN, LEDC_FREQ, LEDC_RESOLUTION);

    for (taskC_timer = 0; taskC_timer < 10; taskC_timer++) {
        int freq = melody[taskC_timer];

        // Output frequency using LEDC
        ledcWrite(BUZZER_PIN, freq);
        Serial.print("Playing note: ");
        Serial.println(freq);

        delay(500);
    }
    ledcWrite(BUZZER_PIN, 0); // Stop tone
}


// Description: Prints A-Z to Serial Monitor.
void taskD() {
    for (char letter = 'A'; letter <= 'Z'; letter++) {
        Serial.print(letter);
        Serial.print(" ");
        delay(250);
    }
    Serial.println();
}


// Function prototypes
void taskA();
void taskB();
//void taskC();
void taskD();

void setup() {
    Serial.begin(115200);
    while(!Serial);

    pinMode(LED_PIN, OUTPUT);

    Wire.begin(8,9);

    lcd.init();
    lcd.backlight();
    delay(2);
    
    ledcAttach(BUZZER_PIN, LEDC_FREQ, LEDC_RESOLUTION);
    
}

void loop() {
    taskA();
    taskB();
    taskC();
    taskD();
}
