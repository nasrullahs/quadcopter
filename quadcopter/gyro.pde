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
  
//  //calculate gyroBias by averaging some values before motors are turned on
//  int gyroReadingsSum[3] = {0, 0, 0};
//  
//  #define NUM_READINGS 50
//  for(int i=0; i<NUM_READINGS; i++) {
//    int gyroReading[3];
//    getGyroRawReading(gyroReading);
//    
//    gyroReadingsSum[0] += gyroReading[0];
//    gyroReadingsSum[1] += gyroReading[1];
//    gyroReadingsSum[2] += gyroReading[2];
//    
//    delay(50);
//  }
//  
//  gyroBias[0] = (float) gyroReadingsSum[0] / (float) NUM_READINGS;
//  gyroBias[1] = (float) gyroReadingsSum[1] / (float) NUM_READINGS;
//  gyroBias[2] = (float) gyroReadingsSum[2] / (float) NUM_READINGS;
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

