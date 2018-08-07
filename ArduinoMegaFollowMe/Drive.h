#ifndef Drive_h
#define Drive_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#endif
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <TinyGPS.h>
#include "MechaQMC5883.h"
#include "Structs.h"

#define SERVOMIN  1280 // this is the 'minimum' pulse length in us
#define SERVOMAX  1680 // this is the 'maximum' pulse length in us
#define MOTORMIN  1150
#define MOTORMAX  1850
#define PULSELENGTH 4.32470703125f  //(17710/4096 -17,714 us per second /12 bits of resolution
#define SERVO 0
#define MOTOR 1
#define SYNC 2

class Drive
{	
public:
	Drive();
  void init(void);
	void set_current_pos(position *current);
	void set_target_pos(position *target);
	void set_vector(vector *toTarget);
	void set_servo(int servo);
  void set_motor(int motor);
  void disable_output();
  void enable_output();
  void set_automatic_motor(bool m);
	void update(MechaQMC5883 compass);
private:
	position current_pos, destination;
	vector toDest;
  servo_data sdata;
  Adafruit_PWMServoDriver pwm;
  TinyGPS gpsmath;
	int manual_motor_speed;
	bool calc_position, eval_compass, calc_motor;
	bool manual_motor;
  bool output_on;
  void write_servo_data();
	void calc_vector();
	void absolut_angle(float angle);
	void relative_angle(int angle);
	void set_speed(int distance);
	void calc_target_vector();
};
#endif

