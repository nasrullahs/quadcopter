#define GYRO_ADDRESS 0x69

float gyroBias[3] = {0, 0, 0};
#define GYRO_SCALE 14.375

void initGyro()
{
  #define ADDR_DLPF_FS 0x16
  #define SCALE_2000_DPS B00011000
  wireWrite(GYRO_ADDRESS, ADDR_DLPF_FS, SCALE_2000_DPS);
  
  //wait for gyro to "spin up"
  delay(100);
}

/*
 * Reads gyroscope x, y, and z values, converts them to deg/sec, and puts the values into 'result'
 * 'result' must be of size 3
 */
void readGyro(float result[])
{
  int gyroReading[3];
  getGyroRawReading(gyroReading);
  
  result[0] = ((float) gyroReading[0] - gyroBias[0]) / GYRO_SCALE;
  result[1] = ((float) gyroReading[1] - gyroBias[1]) / GYRO_SCALE;
  result[2] = ((float) gyroReading[2] - gyroBias[2]) / GYRO_SCALE;
}

void getGyroRawReading(int result[])
{
  #define ADDR_GYRO_XOUT_H 0x1D
  byte buffer[6];
  wireRead(GYRO_ADDRESS, ADDR_GYRO_XOUT_H, 6, buffer);
  
  result[0] = (((int)buffer[0]) << 8 ) | buffer[1];
  result[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  result[2] = (((int)buffer[4]) << 8 ) | buffer[5];
}

