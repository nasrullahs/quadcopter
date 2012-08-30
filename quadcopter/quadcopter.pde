#include <Servo.h>
#include <Wire.h>

//will go away
#define ERRORS_LENGTH 15

#define MOTOR_FRONT_RIGHT 0
#define MOTOR_FRONT_LEFT 1
#define MOTOR_REAR_RIGHT 2
#define MOTOR_REAR_LEFT 3

Servo motors[4];
int errors[ERRORS_LENGTH];//will go away
int head = 0;//will go away

/*
 * Angles of rotation calculated by integrating gyro result
 */
float gyroAngle[3] = {0, 0, 0};

//Variables for kalman filter
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; // angle and angle change rate covariance
float kx, kv;
float gyroVar = 0.1;
float deltaGyroVar = 0.1;
float accelVar = 5;

/*
 * Time (in microseconds, since program started running, read from arduino micros() function)
 * at which last reading was taken.
 */
unsigned long lastReadingTime = 0;
/*
 * Time (in microseconds) between last and second last readings. Used for integrating gyro result.
 */
unsigned long timeDelta = 0;

/*
 * NOTE: the arduino micros() function is used for timing, and it overflows after approximately 70 minutes
 */

void setup() {
  //for debugging
  Serial.begin(9600);
  
  Wire.begin();
  initAccelerometer();
  initGyro();
  
  for(int i=0; i<ERRORS_LENGTH; i++) {
    errors[i] = 0;
  }
  
  //attach and arm motors
  motors[MOTOR_FRONT_RIGHT].attach(11, 1000, 2000);
  motors[MOTOR_FRONT_LEFT].attach(3, 1000, 2000);
  motors[MOTOR_REAR_RIGHT].attach(9, 1000, 2000);
  motors[MOTOR_REAR_LEFT].attach(10, 1000, 2000);
  setSpeed(MOTOR_FRONT_RIGHT, 0);
  setSpeed(MOTOR_FRONT_LEFT, 0);
  setSpeed(MOTOR_REAR_RIGHT, 0);
  setSpeed(MOTOR_REAR_LEFT, 0);
  delay(2000);
}


void loop() {
  float accelAngle[2];
  float prediction[2];
  float gyroRotation[3];
  
  timeDelta = micros() - lastReadingTime;
  lastReadingTime = micros();
  
  getAnglesFromAccel(accelAngle);
  readGyro(gyroRotation);
  
  gyroAngle[0] += gyroRotation[0] * timeDelta / 1000000.0;
  gyroAngle[1] += gyroRotation[1] * timeDelta / 1000000.0;
  gyroAngle[2] += gyroRotation[2] * timeDelta / 1000000.0;
  
  prediction[0] += gyroRotation[0] * timeDelta / 1000000.0;
  prediction[1] += gyroRotation[1] * timeDelta / 1000000.0;
  
  /*
   * NOTE:
   * I still do not understand how kalman filters work, I just copied some
   * code from http://www.den-uijl.nl/electronics/gyro.html
   */
  Pxx += timeDelta * (2 * Pxv + timeDelta * Pvv);
  Pxv += timeDelta * Pvv;
  Pxx += timeDelta * gyroVar;
  Pvv += timeDelta * deltaGyroVar;
  kx = Pxx * (1 / (Pxx + accelVar));
  kv = Pxv * (1 / (Pxx + accelVar));
  
  prediction[0] += (accelAngle[0] - prediction[0]) * kx;
  prediction[1] += (accelAngle[1] - prediction[1]) * kx;
  
  Pxx *= (1 - kx);
  Pxv *= (1 - kx);
  Pvv -= kv * Pxv;
  
  Serial.print(prediction[0]);
  Serial.print(",\t\t");
  Serial.println(prediction[1]);
  
  delay(100);
//  int counter = 0;
//  while(counter < 200) {
//    int x = readAccelerometerX();
//    errors[head] = x;
//    if(head < (ERRORS_LENGTH - 1) ) {
//      head = head + 1;
//    }
//    else {
//      head = 0;
//    }
//    
//    int speed = 40;
//    int proportional = x / 400;
//    
//    
//    long integral = 0;
//    int i = 0;
//    while(i < ERRORS_LENGTH) {
//      integral = integral + errors[i];
//      i = i + 1;
//    }
//    integral = integral / 2000;
//    
//    
//    int errorsDeltaSum = 0;
//    int j = 0;
//    #define DERIVATIVE_HISTORY_LENGTH 4
//    while(j < DERIVATIVE_HISTORY_LENGTH) {
//      int errorIndex1 = (head - j) % ERRORS_LENGTH;
//      int errorIndex2 = (head - j - 1) % ERRORS_LENGTH;
//      errorsDeltaSum = errors[errorIndex1] - errors[errorIndex2];
//      j = j + 1;
//    }
//    int derivative = errorsDeltaSum / DERIVATIVE_HISTORY_LENGTH;
//    derivative = 0;//derivative / 1000;
//    
//    int output = proportional + integral + derivative;
//    
//    if(counter < 2) {
//      output = 0;
//    }
//    
//    Serial.print("proportional ");
//    Serial.print(proportional);
//    Serial.print(",   integral ");
//    Serial.print(integral);
//    Serial.print(",   derivative ");
//    Serial.print(derivative);
//    Serial.print(",   total ");
//    Serial.print(output);
//    Serial.println("");
//    
////    setSpeed(MOTOR_FRONT_RIGHT, speed - output);
////    setSpeed(MOTOR_FRONT_LEFT, speed - output);
////    setSpeed(MOTOR_REAR_RIGHT, speed + output);
////    setSpeed(MOTOR_REAR_LEFT, speed + output);
//    
//    delay(50);
//    
//    counter = counter + 1;
//  }
//  
//  
//  setSpeed(MOTOR_FRONT_RIGHT, 0);
//  setSpeed(MOTOR_FRONT_LEFT, 0);
//  setSpeed(MOTOR_REAR_RIGHT, 0);
//  setSpeed(MOTOR_REAR_LEFT, 0);
//  
//  delay(60000);
}

void setSpeed(int motor, int speed) {
  int fakeAngle = map(speed, 0, 100, 0, 180);
  motors[motor].write(fakeAngle);
}

