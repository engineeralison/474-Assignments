/*
Filename: Lab3Part3Sender.ino
Authors: Shahnaz Mohideen, Alison Tea
Date: 02/18/2025
Description: This file sends a signal to a receiver esp32 when a button is pressed
using an ISR
*/

// Includes
#include <esp_now.h>
#include <WiFi.h>

// Macros
#define BUTTON_PIN 20 // Define the button pin

// Receiver's MAC address
uint8_t broadcastAddress[] = {0x24, 0xEC, 0x4A, 0x0E, 0xB6, 0xE0}; 
volatile bool buttonPressed = false;
volatile unsigned long lastInterruptTime = 0;


// ISR to handle button press 
void IRAM_ATTR buttonInterrupt() {
  unsigned long interruptTime = millis();
    // Debounce check: Ignore button presses that occur within 200ms of each other
    if (interruptTime - lastInterruptTime > 200) {  
        buttonPressed = true;  // Set flag for processing in the main loop
        lastInterruptTime = interruptTime;
    }
}

// Name: onDataSent
// Description: Callback function called when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
 // Check if the delivery was successful and print the status
 Serial0.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");


}


void setup() {
  Serial0.begin(115200);
  Serial0.println("Starting");
  WiFi.mode(WIFI_STA);

 // Set button pin as input and attach an interrupt
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_PIN, buttonInterrupt, FALLING);


 if (esp_now_init() != ESP_OK) return; // Initialize ESP-NOW and check for success
 esp_now_register_send_cb(onDataSent); // Register the send callback function


 esp_now_peer_info_t peerInfo; // Data structure for handling peer information
// Copy the receiver's MAC address to peer information
 memset(&peerInfo, 0, sizeof(peerInfo));
 memcpy(peerInfo.peer_addr, broadcastAddress, 6);  
 peerInfo.channel = 0; // Set WiFi channel to 0 (default)
 peerInfo.encrypt = false; // Disable encryption
 if (esp_now_add_peer(&peerInfo) != ESP_OK) return; // Add peer and check for success
}


void loop() { 

// Check if the button was pressed
  if (buttonPressed) {
    buttonPressed = false; // Reset flag

    // Send data to receiver esp32
    const char *message = "Button Pressed!";
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)message, strlen(message));

    // Print result status
    if (result == ESP_OK) {
      Serial0.println("Message sent successfully");
    } else {
      Serial0.println("Message failed to send");
    }
  }
}
