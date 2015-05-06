#include <Servo.h>
#include <Wire.h>


#define MOTOR_FRONT 1
#define MOTOR_RIGHT 0
#define MOTOR_LEFT 3
#define MOTOR_REAR 2

/*
 * Whether or not motors should be on
 */
boolean powerOn = false;
int baseSpeed = 0;

int desired_angle[2] = {0, 0};

Servo motors[4];

/*
 * Angles of rotation calculated by integrating gyro result
 */
float gyroAngle[3] = {0, 0, 0};

/*
 * Time (in microseconds, since program started running, read from arduino micros() function)
 * at which last reading was taken.
 */
unsigned long lastReadingTime = 0;

//Variables for kalman filter
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; // angle and angle change rate covariance
float gyroVar = 0.1;
float deltaGyroVar = 0.1;
float accelVar = 5.0;

/*
 * Angles of rotation predicted by kalman filter
 */
float anglePrediction[2];

/*
 * Integral of angle for "integral" component of PID controller
 */
float error_integral[2] = {0, 0};

/*
 * NOTE: the arduino micros() function is used for timing, and it overflows after approximately 70 minutes
 */

void setup()
{
  Serial.begin(115200);

  Wire.begin();
  initAccelerometer();
  initGyro();

  //attach and arm motors
  motors[MOTOR_FRONT].attach(10, 1000, 2000);
  motors[MOTOR_RIGHT].attach(3, 1000, 2000);
  motors[MOTOR_LEFT].attach(11, 1000, 2000);
  motors[MOTOR_REAR].attach(9, 1000, 2000);
  setSpeed(MOTOR_FRONT, 0);
  setSpeed(MOTOR_LEFT, 0);
  setSpeed(MOTOR_RIGHT, 0);
  setSpeed(MOTOR_REAR, 0);
  delay(2000);
}


void loop()
{
  if(powerOn)
  {
    float accelAngle[2];
    float gyroRotation[3];
    
    float timeDelta = (float)(micros() - lastReadingTime) / 1000000.0;
    if(lastReadingTime == 0) {
      timeDelta = 0;
    }
    
    lastReadingTime = micros();
    
    getAnglesFromAccel(accelAngle);
    readGyro(gyroRotation);
    
    gyroAngle[0] += gyroRotation[0] * timeDelta;
    gyroAngle[1] += gyroRotation[1] * timeDelta;
    gyroAngle[2] += gyroRotation[2] * timeDelta;
    
    anglePrediction[0] += gyroRotation[0] * timeDelta;
    anglePrediction[1] += gyroRotation[1] * timeDelta;
    
    /*
     * Kalman filter algorithm from http://www.den-uijl.nl/electronics/gyro.html
     */
    Pxx += timeDelta * (2 * Pxv + timeDelta * Pvv);
    Pxv += timeDelta * Pvv;
    Pxx += timeDelta * gyroVar;
    Pvv += timeDelta * deltaGyroVar;
    float kx = Pxx * (1.0 / (Pxx + accelVar));
    float kv = Pxv * (1.0 / (Pxx + accelVar));
    
    anglePrediction[0] += (accelAngle[0] - anglePrediction[0]) * kx;
    anglePrediction[1] += (accelAngle[1] - anglePrediction[1]) * kx;
    
    Pxx *= (1 - kx);
    Pxv *= (1 - kx);
    Pvv -= kv * Pxv;
    
    /*
     * The sensors on the quadcopter are rotated 45 degrees from the axes of the frame,
     * so the frame's rotation needs to be calculated from the sensors' rotation
     */
    float rotation[2];
    rotation[0] = anglePrediction[0] * cos(PI / 4.0) + anglePrediction[1] * sin(PI / 4.0);
    rotation[1] = anglePrediction[0] * sin(PI / 4.0) - anglePrediction[1] * cos(PI / 4.0);
    
    float rotation_derivative[2];
    rotation_derivative[0] = gyroRotation[0] * cos(PI / 4.0) + gyroRotation[1] * sin(PI / 4.0);
    rotation_derivative[1] = gyroRotation[0] * sin(PI / 4.0) - gyroRotation[1] * cos(PI / 4.0);
    
    
    /*
     * PID controller
     */
    float error[2];
    error[0] = rotation[0] - desired_angle[0];
    error[1] = rotation[1] - desired_angle[1];
    
    float proportional[2], integral[2], derivative[2];
    int output[2];
    
    #define GAIN_PROPORTIONAL 0.16
    proportional[0] = error[0] * GAIN_PROPORTIONAL;
    proportional[1] = error[1] * GAIN_PROPORTIONAL;
    
    error_integral[0] += error[0] * timeDelta;
    error_integral[1] += error[1] * timeDelta;
    #define GAIN_INTEGRAL 0.012
    integral[0] = error_integral[0] * GAIN_INTEGRAL;
    integral[1] = error_integral[1] * GAIN_INTEGRAL;
    
    #define GAIN_DERIVATIVE 0.022
    derivative[0] = rotation_derivative[0] * GAIN_DERIVATIVE;
    derivative[1] = rotation_derivative[1] * GAIN_DERIVATIVE;
    
    output[0] = proportional[0] + integral[0] + derivative[0];
    output[1] = proportional[1] + integral[1] + derivative[1];
    
    //safety auto-off if angle gets too high
    if(abs(rotation[0]) > 40 || abs(rotation[1]) > 40) {
      powerOn = false;
      return;
    }
    
    setSpeed(MOTOR_FRONT, baseSpeed + output[1]);
    setSpeed(MOTOR_REAR, baseSpeed - output[1]);
    setSpeed(MOTOR_LEFT, baseSpeed + output[0]);
    setSpeed(MOTOR_RIGHT, baseSpeed - output[0]);
  }
  else
  {
    setSpeed(MOTOR_FRONT, 0);
    setSpeed(MOTOR_REAR, 0);
    setSpeed(MOTOR_LEFT, 0);
    setSpeed(MOTOR_RIGHT, 0);
    
    error_integral[0] = 0;
    error_integral[1] = 0;
  }
}
 
void serialEvent()
{
  char input[4] = {0, 0, 0, 0};
  Serial.readBytes(input, 4);
  
  powerOn = (input[0] == '1');
  baseSpeed = input[1];
  desired_angle[0] = constrain(input[2], -35, 35);
  desired_angle[1] = constrain(input[3], -35, 35);
}

void setSpeed(int motor, int speed)
{
  int fakeAngle = map(speed, 0, 100, 0, 180);
  motors[motor].write(fakeAngle);
}

