/**
 * @file Receiver.ino
 * @author Alison Tea
 * @author Shahnaz Mohideen
 * @date 22-March-2025
 * @brief This program receives sensor data via ESP-NOW and displays an appropriate 
 * @brief alert message on an I2C LCD screen. The system warns of various hazards 
 * @brief such as break-ins, fires, and floods.
 */

// Libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>

/**
 * @brief Sensor types represented as macros for better readability.
 */
#define MOTION 0  ///< Motion detected (break-in alert)
#define TEMP 1    ///< Temperature sensor (not used in this code)
#define HUM 2     ///< Humidity sensor (fire alert)
#define SOUND 3   ///< Sound detected (break-in alert)
#define FLOOD 4   ///< Flood sensor (evacuation alert)

/// LCD display instance, using I2C address 0x27 with 16 columns and 2 rows.
LiquidCrystal_I2C lcd(0x27, 16, 2);

/// Flag indicating whether a message has been received.
volatile bool messageReceived = false;
/// Variable to store the received sensor message.
volatile int sensorMessage;

/**
 * @brief Callback function triggered when ESP-NOW data is received.
 *
 * This function sets the messageReceived flag and copies the received
 * data into the sensorMessage variable.
 *
 * @param esp_now_info Pointer to ESP-NOW metadata.
 * @param incomingData Pointer to received data buffer.
 * @param len Length of the received data.
 */
void IRAM_ATTR dataReceived(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len)
{
    if (len >= sizeof(sensorMessage)) len = sizeof(sensorMessage) - 1; // Prevent overflow
    messageReceived = true;
    memcpy((int*) &sensorMessage, incomingData, len);
    Serial.printf("Data: %d\n", sensorMessage);
}

/**
 * @brief Initializes serial communication, WiFi, LCD, and ESP-NOW.
 */
void setup() {
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Starting\n");

    WiFi.mode(WIFI_STA); // Set WiFi to station mode
    
    // Initialize LCD display
    Wire.begin(8,9);
    lcd.init();
    lcd.backlight();   // Ensure backlight is ON
    lcd.setCursor(0, 0);
    lcd.print("You are safe."); // Default message

    // Initialize ESP-NOW and register the callback function
    if (esp_now_init() != ESP_OK) return;
    esp_now_register_recv_cb(dataReceived);
}

/**
 * @brief Main loop that checks for received messages and updates the LCD display.
 */
void loop() {
    if (messageReceived) {
        lcd.clear();
        lcd.setCursor(0, 0);
        Serial.printf("Sensor Message: %d\n", sensorMessage);
        
        // Display appropriate alert message based on sensor type
        if (sensorMessage == MOTION) {
            lcd.print("BREAK-IN! HIDE!");
        } else if (sensorMessage == HUM) {
            lcd.print("FIRE! RUN!");
        } else if (sensorMessage == SOUND) {
            lcd.print("BREAK-IN! HIDE!");
        } else if (sensorMessage == FLOOD) {
            lcd.print("FLOOD! EVACUATE!");
        }
        
        messageReceived = false;
    }
}
