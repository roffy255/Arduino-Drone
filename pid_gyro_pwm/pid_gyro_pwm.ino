/*following code is built for drone hover testing and
callibaration purpose. Many drone projects face this
initial problem of callibarating esc and tuning pid
constants. this is not a full drone code. only for 
hover testing. (Roll testing)

you can change the throttle by using:
  --bluetooth controller
  --esp wifi kit
  --rf module
  --lora module
  --x-bee
  ......

  place your values from above modules in throttle variable.

  current gyro module ITG3200

  current microcontroller ATMEGA328p

  THIS WORKED FOR ME.. HOPE IT DOES TO YOU TOO :)


*/

#include <Servo.h>
#include <Wire.h>
#include "ITG3200.h"

ITG3200 gyro;
//                                     1
Servo motor_1; //                      |
Servo motor_2; //                      |
Servo motor_3; //               4------ ------2
Servo motor_4; //                      |
//                                     |
//                                     3

int escpin1 = 6;
int escpin2 = 9; //  <------ connect esc in these pwm pins
int escpin3 = 10;
int escpin4 = 11;

float PID, pwmLeft_x, pwmRight_x, error_roll_x, previous_error_roll_x, pwmLeft_y, pwmRight_y, error_roll_y, previous_error_roll_y;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
float ax, ay, az;

double kp = 3.55;  //3.55
double ki = 0.005; //0.003          //adjust these constants to adjust sensitivity
double kd = 2.05;  //2.05

double throttle = 1300;
float desired_angle = 0;

void setup()
{
  Serial.begin(9600);
  motor_1.attach(escpin1);
  motor_2.attach(escpin2);
  motor_3.attach(escpin3);
  motor_4.attach(escpin4);

  startup_esc();

  boost_esc();

  gyro.init();
  gyro.zeroCalibrate(200, 10);
}

void startup_esc()
{
  motor_1.writeMicroseconds(1000);
  motor_2.writeMicroseconds(1000);
  motor_3.writeMicroseconds(1000);
  motor_4.writeMicroseconds(1000);

  delay(10000);
}

void boost_esc()
{
  motor_1.writeMicroseconds(1400);
  motor_2.writeMicroseconds(1400);
  motor_3.writeMicroseconds(1400);
  motor_4.writeMicroseconds(1400);

  delay(3000);
}

void x_axis_call()
{
  error_roll_x = ax - desired_angle;

  pid_p = kp * error_roll_x;

  if (-3 < error_roll_x < 3)
  {
    pid_i = pid_i + (ki * error_roll_x);
  }

  pid_d = kd * (error_roll_x - previous_error_roll_x);

  PID = pid_p + pid_i + pid_d;

  if (PID < -1000)
  {
    PID = -1000;
  }
  if (PID > 1000)
  {
    PID = 1000;
  }

  pwmLeft_x = throttle + PID;
  pwmRight_x = throttle - PID;

  if (pwmRight_x < 1000)
  {
    pwmRight_x = 1000;
  }
  if (pwmRight_x > 2000)
  {
    pwmRight_x = 2000;
  }

  if (pwmLeft_x < 1000)
  {
    pwmLeft_x = 1000;
  }
  if (pwmLeft_x > 2000)
  {
    pwmLeft_x = 2000;
  }
}

void y_axis_call()
{
  error_roll_y = ay - desired_angle;

  pid_p = kp * error_roll_y;

  if (-3 < error_roll_y < 3)
  {
    pid_i = pid_i + (ki * error_roll_y);
  }

  pid_d = kd * (error_roll_y - previous_error_roll_y);

  PID = pid_p + pid_i + pid_d;

  if (PID < -1000)
  {
    PID = -1000;
  }
  if (PID > 1000)
  {
    PID = 1000;
  }

  pwmLeft_y = throttle + PID;
  pwmRight_y = throttle - PID;

  if (pwmRight_y < 1000)
  {
    pwmRight_y = 1000;
  }
  if (pwmRight_y > 2000)
  {
    pwmRight_y = 2000;
  }

  if (pwmLeft_y < 1000)
  {
    pwmLeft_y = 1000;
  }
  if (pwmLeft_y > 2000)
  {
    pwmLeft_y = 2000;
  }
}

void feedMotor()
{
  motor_1.writeMicroseconds(pwmLeft_y);
  motor_2.writeMicroseconds(pwmLeft_x);
  motor_3.writeMicroseconds(pwmRight_y);
  motor_4.writeMicroseconds(pwmRight_x);

  previous_error_roll_x = error_roll_x;
  previous_error_roll_y = error_roll_y;
}

void loop()
{

  gyro.getAngularVelocity(&ax, &ay, &az);
  // Serial.print("Angular Velocity of X , Y , Z: ");
  // Serial.print(ax);
  // Serial.print(" , ");
  // Serial.print(ay);
  // Serial.print(" , ");
  // Serial.print(az);
  // Serial.println(" degrees per second");
  // Serial.println("*************");

  x_axis_call();
  y_axis_call();
  feedMotor();
}
