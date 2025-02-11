#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize the LCD

void setup() {
  Serial.begin(115200);

  Wire.begin(11, 12);
  lcd.init();
  lcd.backlight();
  delay(2);
}


void loop() {
  if (Serial.available()) {
        delay(100); // Wait to receive full input
        String input = Serial.readString();
        Serial.print(input);
        
        
        lcdCommand(0x01);  // Clear display (Equivalent to lcd.clear())
        delay(5);          // Allow LCD to process clear command
        lcdCommand(0x80);  //
        

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(input.substring(0, input.length() - 1));

        /*
        for (int i = 0; i < input.length(); i++) {
          Wire.beginTransmission(0x27);
          uint8_t data = input[i];
          data &= 0xF0;
          data |= 0x01;
          Wire.write(data | 0x08 | 0x04);
          Wire.write(data | 0x08);
          data = input[i];
          data = data << 4;
          data |= 0x01;
          Wire.write(data | 0x08 | 0x04);
          Wire.write(data | 0x08);
          //Wire.write(0x00);
          Wire.endTransmission();
        }
        */
        
    }
    
}

void lcdSend(uint8_t value, uint8_t mode) {
    uint8_t highNibble = (value & 0xF0) | mode | BACKLIGHT;
    uint8_t lowNibble = ((value << 4) & 0xF0) | mode | BACKLIGHT;

    Wire.beginTransmission(LCD_ADDR);
    Wire.write(highNibble | ENABLE);  // High nibble, Enable HIGH
    Wire.write(highNibble);           // High nibble, Enable LOW
    Wire.write(lowNibble | ENABLE);   // Low nibble, Enable HIGH
    Wire.write(lowNibble);            // Low nibble, Enable LOW
    Wire.endTransmission();

    delay(1);  // Small delay for LCD processing
}

void lcdCommand(uint8_t cmd) {
    lcdSend(cmd, COMMAND);
    delay(2);  // Ensure command processes correctly
}