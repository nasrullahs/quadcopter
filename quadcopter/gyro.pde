#include <Wire.h>

#define GYRO_ADDRESS 0x69

void initGyro() {
#define ADDR_DLPF_FS 0x16
#define SCALE_2000_DPS B00011000
  wireWrite(GYRO_ADDRESS, ADDR_DLPF_FS, SCALE_2000_DPS);
}

int readGyroX() {
  Wire.beginTransmission(GYRO_ADDRESS);

#define ADDR_GYRO_XOUT_H 0x1D
  Wire.send(ADDR_GYRO_XOUT_H);
  Wire.endTransmission();

  Wire.requestFrom(GYRO_ADDRESS, 2);
  int xh = 0, xl = 0;
  if(Wire.available() == 2) {
    xh = Wire.receive();
    xl = Wire.receive();
  }

  xh = xh << 8;
  int x = xh | xl;
  return x;
}

int readGyroY() {
  Wire.beginTransmission(GYRO_ADDRESS);

#define ADDR_GYRO_YOUT_H 0x1F
  Wire.send(ADDR_GYRO_YOUT_H);
  Wire.endTransmission();

  Wire.requestFrom(GYRO_ADDRESS, 2);
  int yh = 0, yl = 0;
  if(Wire.available() == 2) {
    yh = Wire.receive();
    yl = Wire.receive();
  }

  yh = yh << 8;
  int y = yh | yl;
  return y;
}

int readGyroZ() {
    Wire.beginTransmission(GYRO_ADDRESS);

#define ADDR_GYRO_ZOUT_H 0x21
  Wire.send(ADDR_GYRO_ZOUT_H);
  Wire.endTransmission();

  Wire.requestFrom(GYRO_ADDRESS, 2);
  int zh = 0, zl = 0;
  if(Wire.available() == 2) {
    zh = Wire.receive();
    zl = Wire.receive();
  }

  zh = zh << 8;
  int z = zh | zl;
  return z;
}
