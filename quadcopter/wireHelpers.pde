#include <Wire.h>

void wireWrite(char address, char registerAddress, char data) {
  Wire.beginTransmission(address);
  Wire.send(registerAddress);
  Wire.send(data);
  Wire.endTransmission();
}

unsigned char wireRead(char address, char registerAddress)  {
  unsigned char data = 0;
  Wire.beginTransmission(address);
  Wire.send(registerAddress);
  #define NUM_BYTES 1
  Wire.requestFrom(address, NUM_BYTES);
  
  if(Wire.available()) {
    data = Wire.receive();
  }

  Wire.endTransmission();
  
  return data;
}
