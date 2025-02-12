#include <esp_now.h>
#include <WiFi.h>


#define BUTTON_PIN 20 // Define the button pin

// Replace with receiver's MAC address (correct format)
uint8_t broadcastAddress[] = {0x24, 0xEC, 0x4A, 0x0E, 0x81, 0xF8}; 

volatile bool buttonPressed = false; // Flag to track button press

// ISR to handle button press
void IRAM_ATTR buttonISR() {
  buttonPressed = true; // Set flag when button is pressed
}

// Callback function when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

void setup() {
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Set button pin as input with internal pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Attach an interrupt to the button
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(onDataSent); // Register the send callback function

  // Configure peer information
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
}

void loop() {
  
  // Check if the button was pressed
  if (buttonPressed) {
    buttonPressed = false; // Reset flag

    // Example data to send
    const char *message = "Button Pressed!";
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)message, strlen(message));

    // Print result status
    if (result == ESP_OK) {
      Serial.println("Message sent successfully");
    } else {
      Serial.println("Message failed to send");
    }
  }
  
}