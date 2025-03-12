#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>

#define MOTION 0
#define TEMP 1
#define HUM 2
#define SOUND 3
#define FLOOD 4


LiquidCrystal_I2C lcd(0x27, 16, 2);
volatile bool messageReceived = false;
volatile uint8_t sensor = 0;


void IRAM_ATTR dataReceived(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len)
{
// =========> TODO: This callback function will be invoked when signal is received
// 			over ESP-NOW. Implement the necessary functionality that will 
//			trigger the message to the LCD.
//
  messageReceived = true;
  sensor = *incomingData;
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
    if(sensor == MOTION){
      lcd.print("Motion");
    }else if (sensor = TEMP){
      lcd.print("Temperature");
    }else if (sensor = HUM){
      lcd.print("Humidity");
    }else if (sensor == SOUND){
      lcd.print("Sound");
    }else if (sensor == FLOOD){
      lcd.print("Water");
    }
    delay(2000);
    messageReceived = false;
  }

  
}
