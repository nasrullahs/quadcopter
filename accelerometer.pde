#include <Wire.h>

#define ACCELEROMETER_ADDRESS 0x40

void initAccelerometer() {
  #define ADDR_OFFSET_LSB1 0x35
  #define RANGE_1_5_G B0010
  wireWrite(ACCELEROMETER_ADDRESS, ADDR_OFFSET_LSB1, RANGE_1_5_G);
  
  #define ADDR_BW_TCS 0x20
  //TODO : increase bandwidth, as 20Hz is too slow
  #define FILTER_BW_20 B00000001
  wireWrite(ACCELEROMETER_ADDRESS, ADDR_BW_TCS, FILTER_BW_20);
}

int readAccelerometerX() {
  Wire.beginTransmission(ACCELEROMETER_ADDRESS);
  
  #define ADDR_ACC_X_LSB 0x02
  Wire.send(ADDR_ACC_X_LSB);
  Wire.endTransmission();
  
  Wire.requestFrom(ACCELEROMETER_ADDRESS, 2);
  int xh=0, xl=0;
  if(Wire.available() == 2) {
    xl = Wire.receive();
    xh = Wire.receive();
  }
  
  int x;
  xl = xl & B11111100;
  xh = xh << 8;
  x = xh | xl;
  x = x >> 2;
  
  return x;
}

int readAccelerometerY() {
  Wire.beginTransmission(ACCELEROMETER_ADDRESS);  
  
  #define ADDR_ACC_Y_LSB 0x04
  Wire.send(ADDR_ACC_Y_LSB);
  Wire.endTransmission();
  
  Wire.requestFrom(ACCELEROMETER_ADDRESS, 2);
  int yh=0, yl=0;
  if(Wire.available() == 2) {
    yl = Wire.receive();
    yh = Wire.receive();
  }
  
  int y;
  yl = yl & B11111100;
  yh = yh << 8;
  y = yh | yl;
  y = y >> 2;
  
  return y;
}

