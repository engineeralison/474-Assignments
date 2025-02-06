#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize the LCD

void setup() {
  Serial.begin(115200);

  Wire.begin(11, 12);
  lcd.init();
  //delay(2);
  lcd.backlight();
}


void loop() {
  lcd.setCursor(0, 0);
  // print message
  lcd.print("Hello, World!");
  delay(1000);
  /*
  if (Serial.available()) {
        delay(100); // Wait to receive full input
        String input = Serial.readString();
        Serial.print(input);
        
        Wire.beginTransmission(0x27);
        Wire.write(0x05);
        Wire.write(0x00);
        Wire.write(0x84);
        Wire.write(0x00);
        
        Wire.endTransmission();

        
        for (int i = 0; i < input.length(); i++) {
          uint8_t data = input[i];
          data &= 0xF0;
          data |= 0x02;
          Wire.write(data);
          Wire.write(0x00);
          data = input[i];
          data = data << 4;
          data |= 0x02;
          Wire.write(data);
          Wire.write(0x00);
        }

        Wire.endTransmission();
        
        
    }
    */
}
/*
void getInput(char *str) {
  uint8_t i = 0;

  while (true) {
    if (Serial.available()) {
      str[i] = Serial.read();

      // break if we received newline
      if (str[i] == '\n'){
        break;
      }

      // only increment i if there's still space
      if (i < 16){
        i++;
      }
    }
  }
  str[i] = '\0';
}
*/
