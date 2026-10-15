#include <Arduino.h>
#include <Wire.h>
#include "pins.h"   // I2C_SDA, I2C_SCL

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.printf("SDA=GPIO%d SCL=GPIO%d\n", (int)I2C_SDA, (int)I2C_SCL);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
}

void loop() {
  Serial.println("Scanning...");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Found 0x%02X\n", a);
      found++;
    }
  }
  Serial.printf("Done, %d device(s)\n\n", found);
  delay(2000);
}
