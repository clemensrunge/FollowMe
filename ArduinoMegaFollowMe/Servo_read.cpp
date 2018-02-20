#include "Servo_read.h" 

volatile unsigned long Servo_Motor[2];
volatile unsigned long Servo_Servo[2];

void ISR_MotorReciever(){
  if(digitalRead(ReceiverIntPinMotor)){
    Servo_Motor[0] = micros();
  }else{
    Servo_Motor[1] = micros() - Servo_Motor[0];
  }
}

void ISR_ServoReciever(){
  if(digitalRead(ReceiverIntPinServo)){
    Servo_Servo[0] = micros();
  }else{
    Servo_Servo[1] = micros() - Servo_Servo[0];
  }
}

Servo_read::Servo_read()
{
  attachInterrupt(digitalPinToInterrupt(ReceiverIntPinMotor), ISR_MotorReciever, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ReceiverIntPinServo), ISR_ServoReciever, CHANGE);
}

void Servo_read::get_servo_data(servo_data *sdata)
{
  if(2400 > Servo_Servo[1])
  {
    sdata->servo = map(Servo_Servo[1], 1000, 2000, -100, 100);
  }
  if(2400 > Servo_Motor[1])
  {
    sdata->motor = map(Servo_Motor[1], 1000, 2000, -100, 100);
  }
}

