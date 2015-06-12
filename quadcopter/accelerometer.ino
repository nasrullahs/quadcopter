#define ACCELEROMETER_ADDRESS 0x40

float accelBias[3] = {0, 0, 0};
#define ACCEL_SCALE 4095.5

void initAccelerometer()
{
  byte buffer[1];
  
  //the ee_w bit must be set to write to configuration registers (not only for writing to EEPROM)
  #define ADDR_CTRL_REG0 0x0D
  #define EE_W_MASK B00010000
  wireRead(ACCELEROMETER_ADDRESS, ADDR_CTRL_REG0, 1, buffer);
  byte ctrl_reg0_value = buffer[0] | EE_W_MASK;
  wireWrite(ACCELEROMETER_ADDRESS, ADDR_CTRL_REG0, ctrl_reg0_value);
  
  #define ADDR_BW_TCS 0x20
  #define BW_MASK B11110000
  #define FILTER_BW_300 B01010000
  wireRead(ACCELEROMETER_ADDRESS, ADDR_BW_TCS, 1, buffer);
  byte bw_tcs_value = (buffer[0] & ~BW_MASK) | FILTER_BW_300;
  wireWrite(ACCELEROMETER_ADDRESS, ADDR_BW_TCS, bw_tcs_value);
  
  ctrl_reg0_value = ctrl_reg0_value & ~EE_W_MASK;
  wireWrite(ACCELEROMETER_ADDRESS, ADDR_CTRL_REG0, ctrl_reg0_value);
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

