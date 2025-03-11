#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"

Adafruit_AM2320 am2320 = Adafruit_AM2320();

void setup() {
  Serial.begin(115200); // Use a higher baud rate for ESP32
  while(!Serial);
  delay(1000); // Give time for Serial to start

  Serial.println("Initializing AM2320 sensor...");

  // Initialize I2C (for ESP32 default SDA=21, SCL=22)
  Wire.begin(8,9); 

  if (!am2320.begin()) {
    Serial.println("Failed to detect AM2320 sensor! Check wiring.");
    while (1) { delay(10); } // Stop execution if sensor isn't found
  }

  Serial.println("AM2320 sensor found!");
  am2320.begin();
}

void loop() {
  Serial.print("Temp: ");
  Serial.print(am2320.readTemperature());
  Serial.print(" C\t");

  Serial.print("Humidity: ");
  Serial.print(am2320.readHumidity());
  Serial.println(" %");

  delay(2000);
}
