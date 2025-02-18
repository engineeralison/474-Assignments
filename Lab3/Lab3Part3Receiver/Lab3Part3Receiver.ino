#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>


LiquidCrystal_I2C lcd(0x27, 16, 2);
volatile bool messageReceived = false;
volatile bool count_increased = false;
volatile int count = 0;
hw_timer_t * timer = NULL; // Declare a timer variable and initialize to null


void IRAM_ATTR dataReceived(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len)
{
// =========> TODO: This callback function will be invoked when signal is received
// 			over ESP-NOW. Implement the necessary functionality that will 
//			trigger the message to the LCD.
//
  messageReceived = true;
}


// =========> TODO: Write your timer ISR here.
void IRAM_ATTR timerInterrupt() {
  if(messageReceived){
    count++;
    count_increased = true;
  }
}


void setup() {

 Serial0.begin(115200);
 Serial0.println("Starting\n");
 WiFi.mode(WIFI_STA);
// =========> TODO: Initialize LCD display
Wire.begin(8,9);
lcd.init();
lcd.backlight();   // Ensure backlight is ON
lcd.setCursor(0, 0);
lcd.print("Count: ");
lcd.print(count);

// =========> TODO: create a timer, attach an interrupt, set an alarm which will
//			update the counter every second.

timer = timerBegin(100000); // prescaler 100000
// Attach timerInterrupt function to the timer
timerAttachInterrupt(timer, &timerInterrupt); 
// Set alarm to trigger interrupt every second, repeating (true), 
// number of autoreloads (0=unlimited)
timerAlarm(timer, 100000, true, 0);


 // Initializes ESP-NOW and check if it was successful; if not, exit the setup function
 if (esp_now_init() != ESP_OK) return;
 // Registers the callback function 'dataReceived' to be called when data is received   
 // via ESP-NOW
 esp_now_register_recv_cb(dataReceived);
}


void loop() {
// =========> TODO: Print out an incrementing counter to the LCD.
//			If a signal has been received over ESP-NOW, print out “New
// 			Message!” on the LCD

  if(messageReceived){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("New Message!");
    delay(2000);
    messageReceived = false;
  }

  if(count_increased){
    count_increased = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Count: ");
    lcd.print(count);
  }
}
