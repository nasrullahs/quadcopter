#include <Wire.h>

/*
 * Writes 'data' to 'registerAddress' of I2C device with address 'deviceAddress'
 */
void wireWrite(char deviceAddress, char registerAddress, byte data)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();
}

/*
 * Reads 'numBytes' bytes from I2C device with address 'deviceAddress' from register 'registerAddress'
 * and puts result in 'result'
 */
void wireRead(byte deviceAddress, byte registerAddress, int numBytes, byte result[])
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission();

  Wire.requestFrom((int)deviceAddress, numBytes);

  int i = 0;
  
  while(Wire.available())
  {
    result[i] = Wire.read();
    i++;
  }
}

