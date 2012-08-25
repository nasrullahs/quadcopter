#include <Wire.h>

#define GYRO_ADDRESS 0x69

float gyroBias[3] = {0, 0, 0};
float gyroAngle[3] = {0, 0, 0};
/*
 * Time (in microseconds, since program started running, read from arduino micros() function)
 * at which last reading was taken.
 */
unsigned long lastReadingTime = 0;
/*
 * Time (in microseconds) between last and second last readings
 */
unsigned long lastReadingTimeDelta = 0;

/*
 * NOTE: the arduino micros() function is used for timing, and it overflows after approximately 70 minutes
 */

void initGyro() {
  #define ADDR_DLPF_FS 0x16
  #define SCALE_2000_DPS B00011000
  wireWrite(GYRO_ADDRESS, ADDR_DLPF_FS, SCALE_2000_DPS);
}

/*
 * Reads gyro and calculates angles of rotation about x, y, and z axes
 */
void getAnglesFromGyro(float result[]) {
  float rotation[3];
  readGyro(rotation);
  
  gyroAngle[0] = gyroAngle[0] + rotation[0] * lastReadingTimeDelta/1000000.0;
  gyroAngle[1] = gyroAngle[1] + rotation[1] * lastReadingTimeDelta/1000000.0;
  gyroAngle[2] = gyroAngle[2] + rotation[2] * lastReadingTimeDelta/1000000.0;
  
  result[0] = gyroAngle[0];
  result[1] = gyroAngle[1];
  result[2] = gyroAngle[2];
}

/*
 * Reads gyroscope x, y, and z values, converts them to deg/sec, and puts the values into 'result'
 * 'result' must be of size 3
 */
void readGyro(float result[]) {
  #define ADDR_GYRO_XOUT_H 0x1D
  byte buffer[6];
  wireRead(GYRO_ADDRESS, ADDR_GYRO_XOUT_H, 6, buffer);
  lastReadingTimeDelta = micros() - lastReadingTime;
  lastReadingTime = micros();
  
  int gyroReading[3];
  gyroReading[0] = (((int)buffer[0]) << 8 ) | buffer[1];
  gyroReading[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  gyroReading[2] = (((int)buffer[4]) << 8 ) | buffer[5];
  
  #define GYRO_SCALE 14.375
  result[0] = ((float) gyroReading[0] - gyroBias[0]) / GYRO_SCALE;
  result[1] = ((float) gyroReading[1] - gyroBias[1]) / GYRO_SCALE;
  result[2] = ((float) gyroReading[2] - gyroBias[2]) / GYRO_SCALE;
}

