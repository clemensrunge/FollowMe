#ifndef Drive_h
#define Drive_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#endif
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <TinyGPS.h>
#include <MechaQMC5883.h>
#include "Structs.h"

#define SERVOMIN  1330 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  1630 // this is the 'maximum' pulse length count (out of 4096)
#define MOTORMIN  1150
#define MOTORMAX  1850
#define SERVO 0
#define MOTOR 1
#define SYNC 2

class Drive
{	
public:
	Drive();
	void set_current_pos(position current);
	void set_target_pos(position target);
	void set_vector(vector toTarget);
	void set_servo(byte servo);
  void set_motor(byte motor);  
	void update();
private:
	position current_pos, destination;
	vector toDest;
  servo_data sdata;
  Adafruit_PWMServoDriver pwm;
  TinyGPS gpsmath;
  MechaQMC5883 compass;
	int manual_motor_speed;
	bool calc_position, eval_compass, calc_motor;
	bool manual_motor;
  void write_servo_data();
	void calc_vector();
	void absolut_angle(float angle);
	void relative_angle(int angle);
	void set_speed(int distance);
	void calc_target_vector();
};
#endif

