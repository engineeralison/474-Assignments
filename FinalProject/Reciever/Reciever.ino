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
 volatile int sensorMessage;

// Name: dataRecieved
// Description: This callback function will be invoked when signal is received
// 			        over ESP-NOW. Implement the necessary functionality that will 
//			        trigger the message to the LCD.
//
void IRAM_ATTR dataReceived(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len)
{

  if (len >= sizeof(sensorMessage)) len = sizeof(sensorMessage) - 1; // Prevent overflow
  
  messageReceived = true;
  memcpy((int*) &sensorMessage,incomingData, len);
  // sensorMessage[len] = '\0';

  Serial.printf("Data: %d\n",  incomingData );
  
}

void setup() {

 Serial.begin(115200);
 while(!Serial);


 Serial.println("Starting\n");
 WiFi.mode(WIFI_STA);
 // Initialize LCD display
 Wire.begin(8,9);
 lcd.init();
 lcd.backlight();   // Ensure backlight is ON
 lcd.setCursor(0, 0);

  // Write welcome message to the alarm
  lcd.print("You are safe.");
 // Initializes ESP-NOW and check if it was successful; if not, exit the setup function
 if (esp_now_init() != ESP_OK) return;
 // Registers the callback function 'dataReceived' to be called when data is received   
 // via ESP-NOW
 esp_now_register_recv_cb(dataReceived);
}

void loop() {
  */
   if(messageReceived){
    lcd.clear();
    lcd.setCursor(0, 0);
    Serial.printf("Sensor Message: %d\n", sensorMessage);
    if(sensorMessage == MOTION){
      lcd.print("BREAK-IN! HIDE!");
    }else if (sensorMessage == HUM){
      lcd.print("FIRE! RUN!");
    }else if (sensorMessage == SOUND){
      lcd.print("BREAK-IN! HIDE!"");
    }else if (sensorMessage == FLOOD){
      lcd.print("FLOOD! EVACUATE!");
    }
    // delay(2000);
    messageReceived = false;
  }


}
