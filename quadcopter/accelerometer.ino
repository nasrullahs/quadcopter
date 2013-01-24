#define ACCELEROMETER_ADDRESS 0x40

float accelBias[3] = {0, 0, 0};
#define ACCEL_SCALE 4095.5

void initAccelerometer()
{
  #define ADDR_OFFSET_LSB1 0x35
  #define RANGE_2_G B0100
  wireWrite(ACCELEROMETER_ADDRESS, ADDR_OFFSET_LSB1, RANGE_2_G);
  
  #define ADDR_BW_TCS 0x20
  #define FILTER_BW_1200 B00000111
  wireWrite(ACCELEROMETER_ADDRESS, ADDR_BW_TCS, FILTER_BW_1200);
  
//  //calculate accelBias by averaging some values before motors are turned on
//  int accelReadingsSum[3] = {0, 0, 0};
//  
//  #define NUM_READINGS 50
//  for(int i=0; i<NUM_READINGS; i++) {
//    int accelReading[3];
//    getAccelRawReading(accelReading);
//    
//    accelReadingsSum[0] += accelReading[0];
//    accelReadingsSum[1] += accelReading[1];
//    accelReadingsSum[2] += accelReading[2] - ACCEL_SCALE;
//    
//    delay(50);
//  }
//  
//  accelBias[0] = (float) accelReadingsSum[0] / (float) NUM_READINGS;
//  accelBias[1] = (float) accelReadingsSum[1] / (float) NUM_READINGS;
//  accelBias[2] = (float) accelReadingsSum[2] / (float) NUM_READINGS;
}

/*
 * Reads accelerometer and calculates angles of rotation about x and y axes (in degrees). Puts result in 'result'
 * 'result' must be of size 2
 */
void getAnglesFromAccel(float result[])
{
  float acceleration[3];
  readAccelerometer(acceleration);
  
  result[0] = atan2(acceleration[2], acceleration[1]) * -360.0 / (2*PI) + 90.0;
  result[1] = atan2(acceleration[2], acceleration[0]) * 360.0 / (2*PI) - 90.0;
}

/*
 * Reads accelerometer x, y, and z values, converts them to g's, and puts them into 'result'
 * 'result' must be of size 3
 */
void readAccelerometer(float result[])
{
  int accelReading[3];
  getAccelRawReading(accelReading);
  
  result[0] = ((float) (accelReading[0] - accelBias[0])) / ACCEL_SCALE;
  result[1] = ((float) (accelReading[1] - accelBias[1])) / ACCEL_SCALE;
  result[2] = ((float) (accelReading[2] - accelBias[2])) / ACCEL_SCALE;
}


void getAccelRawReading(int result[])
{
  #define ADDR_ACC_X_LSB 0x02
  byte buffer[6];
  wireRead(ACCELEROMETER_ADDRESS, ADDR_ACC_X_LSB, 6, buffer);
  
  result[0] = combine(buffer[0], buffer[1]);
  result[1] = combine(buffer[2], buffer[3]);
  result[2] = combine(buffer[4], buffer[5]);
}

int combine(int low, int high)
{
  int result = high << 8 | low;
  result = result >> 2;
  return result;
}

