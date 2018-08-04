#ifndef Servo_read_h
#define Servo_read_h


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#endif

#include "Structs.h"

#define ReceiverIntPinMotor 2
#define ReceiverIntPinServo 3

class Servo_read
{ 
  bool no_signal; 
public:
  void init();
  void get_servo_data(servo_data *sdata);
};
#endif

