/*
Filename: Lab3Part1
Authors: Shahnaz Mohideen, Alison Tea
Date: 02/20/2025
Description: This module takes in a string input in the serial moniter and prints
it out on the lcd display using the Wire library
*/

// Includes
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Global Variable
LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize the LCD

// Function Prototypes
void sendCommand(uint8_t);
void sendData(uint8_t);

void setup() {
  Serial.begin(115200);
  
  Wire.begin(11,12);
  lcd.init();
  lcd.backlight();

  delay(2);
}

void loop() {
    if (Serial.available()) {
        sendCommand(0x01);  // Clear display
        sendCommand(0x80);  // Set cursor to beginning
        delay(5);

        while (Serial.available()) {
            char c = Serial.read();
            if(c != '\n'){
              sendData(c);
            }
        }
    }
}

// Name: sendCommand
// Description: Takes a command and sends it to the lcd uisng the 
// Wire library
void sendCommand(uint8_t command) {
    Wire.beginTransmission(0x27);
    Wire.write((command & 0xF0) | 0x08); // Send high nibble with backlight on
    Wire.write((command & 0xF0) | 0x0C); // Enable bit high
    Wire.write((command & 0xF0) | 0x08); // Enable bit low
    
    Wire.write((command << 4) | 0x08); // Send low nibble with backlight on
    Wire.write((command << 4) | 0x0C); // Enable bit high
    Wire.write((command << 4) | 0x08); // Enable bit low
    Wire.endTransmission();
    delay(2);
}

// Name: sendData
// Description: Takes in a char and writes it to the lcd display
void sendData(uint8_t data) {
    Wire.beginTransmission(0x27);
    Wire.write((data & 0xF0) | 0x09); // Send high nibble with RS=1, Enable=0
    Wire.write((data & 0xF0) | 0x0D); // Enable bit high
    Wire.write((data & 0xF0) | 0x09); // Enable bit low
    
    Wire.write((data << 4) | 0x09); // Send low nibble with RS=1, Enable=0
    Wire.write((data << 4) | 0x0D); // Enable bit high
    Wire.write((data << 4) | 0x09); // Enable bit low
    Wire.endTransmission();
    delay(2);
}