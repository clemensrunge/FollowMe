#include "Servo_read.h" 

volatile unsigned long Servo_Motor;
volatile unsigned long Last_Servo_Motor;
volatile unsigned long Servo_Steering;
volatile unsigned long Last_Signal;

void ISR_MotorReciever(){
  unsigned long temp_micros = micros();
  static unsigned long motor_start;
  if(digitalRead(ReceiverIntPinMotor)){
    motor_start = temp_micros;
  }else{
    Last_Servo_Motor = Servo_Motor;
    Servo_Motor = temp_micros - motor_start;
  }
  Last_Signal = temp_micros;
}

void ISR_ServoReciever(){
  unsigned long temp_micros = micros();
  static unsigned long steering_start;
  if(digitalRead(ReceiverIntPinServo)){
    steering_start = temp_micros;
  }else{
    Servo_Steering = temp_micros - steering_start;
  }
  Last_Signal = temp_micros;
}

void Servo_read::init(void)
{
  Servo_Motor = 0;
  Last_Servo_Motor = 0;
  Servo_Steering = 0;
  no_signal = false;
  attachInterrupt(digitalPinToInterrupt(ReceiverIntPinMotor), ISR_MotorReciever, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ReceiverIntPinServo), ISR_ServoReciever, CHANGE);
}

void Servo_read::get_servo_data(servo_data *sdata)
{
  unsigned long temp = Servo_Steering;
  if( 60000 < micros() - Last_Signal)
  {
    if(false == no_signal)
      {
        Serial.println("[ERROR] No Reciever Signal for longer than 60ms.");
      }
      no_signal = true;
      sdata->servo = 0;
      sdata->motor = 0;
  }
  else
  {
    no_signal = false;
    if(2100 < temp || temp < 900)
    {
      sdata->servo = 0;
    }
    else
    {      
      sdata->servo = map(temp, 1000, 2000, -100, 100);
    }
    temp = Servo_Motor;
    if(2100 < temp || temp < 900)
    {
      sdata->motor = 0;      
    }
    else
    {
      int last_motor =  map(Last_Servo_Motor, 1000, 2000, -100, 100);
      sdata->motor = map(temp, 1000, 2000, -100, 100);
      if(0 < sdata->motor)
      {
        if(last_motor < sdata->motor -30)
        {
          sdata->motor = 0;
        }
      }
      else
      {
        if(last_motor > sdata->motor +30)
        {
          sdata->motor = 0;
        }
      }
    }
  }
}

