#include <Servo.h>
#include <Wire.h>
#include "ITG3200.h"

ITG3200 gyro;

Servo motor_1;
Servo motor_2;
Servo motor_3;
Servo motor_4;

int escpin1 = 6;
int escpin2 = 9;
int escpin3 = 10;
int escpin4 = 11;

float PID, pwmLeft, pwmRight, error_roll, previous_error_roll;
float pid_p=0;
float pid_i=0;
float pid_d=0;

double kp=3.55;//3.55
double ki=0.005;//0.003
double kd=2.05;//2.05

double throttle=1300; 
float desired_angle = 0;


void setup() 
{
  Serial.begin(9600);
  motor_1.attach(escpin1);
  motor_2.attach(escpin2);
  motor_3.attach(escpin3);
  motor_4.attach(escpin4);

  motor_1.writeMicroseconds(1000);
  motor_2.writeMicroseconds(1000);
  motor_3.writeMicroseconds(1000);
  motor_4.writeMicroseconds(1000);

  delay(10000);

   gyro.init();
   gyro.zeroCalibrate(200,10);
}

void loop() {
 
 float ax,ay,az;
    gyro.getAngularVelocity(&ax,&ay,&az);
    Serial.print("Angular Velocity of X , Y , Z: ");
    Serial.print(ax);
    Serial.print(" , ");
    Serial.print(ay);
    Serial.print(" , ");
    Serial.print(az);
    Serial.println(" degrees per second");
    Serial.println("*************");
    delay(1000);

    error_roll = ax - desired_angle;

    pid_p = kp*error_roll;

    if(-3 < error_roll < 3)
    {
       pid_i = pid_i+(ki*error_roll);
    }

    pid_d = kd*(error_roll - previous_error_roll);

    PID = pid_p + pid_i + pid_d;

    if(PID < -1000)
    {
      PID=-1000;
    }
    if(PID > 1000)
    {
      PID=1000;
    }

    pwmLeft = throttle + PID;
    pwmRight = throttle - PID;

    
if(pwmRight < 1000)
{
  pwmRight= 1000;
}
if(pwmRight > 2000)
{
  pwmRight=2000;
}

if(pwmLeft < 1000)
{
  pwmLeft= 1000;
}
if(pwmLeft > 2000)
{
  pwmLeft=2000;
}


motor_2.writeMicroseconds(pwmLeft);
motor_4.writeMicroseconds(pwmRight);
previous_error_roll = error_roll;

}
