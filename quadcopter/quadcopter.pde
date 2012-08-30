#include <Servo.h>
#include <Wire.h>

#define ERRORS_LENGTH 15

#define MOTOR_FRONT_RIGHT 0
#define MOTOR_FRONT_LEFT 1
#define MOTOR_REAR_RIGHT 2
#define MOTOR_REAR_LEFT 3

Servo motors[4];
int errors[ERRORS_LENGTH];
int head = 0;

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
  
  
  for(int i=0; i<3; i++) {
    float buffer[3];
    readGyro(buffer);
  }
}


void loop() {
  float gyroResult[3];
  float accelResult[3];
  getAnglesFromAccel(accelResult);
  getAnglesFromGyro(gyroResult);
  Serial.print(gyroResult[0]);
  Serial.print(", ");
  Serial.print(accelResult[0]);
  Serial.print(",\t\t");
  Serial.print(gyroResult[1]);
  Serial.print(", ");
  Serial.print(accelResult[1]);
  Serial.print(",\t\t");
  Serial.println(gyroResult[2]);
  //Serial.print(", ");
  //Serial.println(accelResult[2]);
  
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

