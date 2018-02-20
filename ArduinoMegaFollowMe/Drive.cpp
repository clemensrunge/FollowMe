#include "Drive.h"

Drive::Drive()
{
	Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
	TinyGPS gpsmath = TinyGPS();
	MechaQMC5883 compass = MechaQMC5883();
	current_pos.lat = 0.0f;
	current_pos.lon = 0.0f;
	destination.lat = 0.0f;
	destination.lon = 0.0f;
	manual_motor = false;
	manual_motor_speed = 0;
	compass.init(); //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
	pwm.begin();
	pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates	
	pwm.setPWM(SYNC, 0, 463); //ca. 2ms SYNC pulse
  sdata.servo = 0;
  sdata.motor = 0;
  write_servo_data();
  calc_motor = false;  
  calc_position = false;
  eval_compass = false;  
}

void Drive::set_current_pos(position current)
{
	current_pos = current;
	if(destination.lat != 0.0)
	{
		eval_compass = true;
    calc_position = true;
	}
}

void Drive::set_target_pos(position target)
{
	destination = target;
	if(current_pos.lat != 0.0)
	{
		eval_compass = true;
    calc_position = true;
	}
}

void Drive::set_vector(vector to_target)
{
  calc_position = false;
  eval_compass = true;  
}

void Drive::set_servo(byte servo)
{
  eval_compass = false;
  calc_position = false;
  sdata.servo = servo;
}

void Drive::set_motor(byte motor)
{
  calc_motor = false;
  sdata.motor = motor;
}

void Drive::update()
{
  int x, y, z;
  int azimuth;
  int pulse;
  int direc = 0;
  int nextAngle;
  
  if(calc_position) {
   toDest.angle = gpsmath.course_to(current_pos.lat, current_pos.lon, destination.lat, destination.lon);
   toDest.distance = gpsmath.distance_between(current_pos.lat, current_pos.lon, destination.lat, destination.lon);
  }

  if(eval_compass)
  {
    // Kompass auslesen
    compass.read(&x, &y, &z, &azimuth);
    //servo daten berechnen
    nextAngle = toDest.angle - azimuth; 
    if(nextAngle < -180){
      nextAngle =+ 360;
    }
      else if(nextAngle > 180) {
      nextAngle =- 360;
      }
      //rechts
      if(0 <= nextAngle){
        if(nextAngle < 5){ 
          pulse = map(nextAngle,0, 5,40,100);  
        }else{
          pulse = 100;
        }
      } 
      //links
      else{
        if(nextAngle > -5){ 
          pulse = map(nextAngle, 0, -5, -40 , -100);  
        }else{ 
          pulse = -100;
        }
      }
      sdata.servo = pulse;
  }
  
  
  if(calc_motor)
  {
    if(toDest.distance > 2){
      pulse = 20;
    }
    if(toDest.distance <= 1){
      pulse = 0;
    }
    if(toDest.distance > 1 && toDest.distance <= 2){
      pulse = map(toDest.distance, 1, 2, 10, 20);
    }
    sdata.motor = pulse;
  }	
  write_servo_data();
}

void Drive::write_servo_data()
{
  double pulselength;
  pulselength = 17710;   // 17,710 us per second 
  pulselength /= 4096;  // 12 bits of resolution
  // Servo
  sdata.servo = map(sdata.servo, -100, 100, SERVOMAX, SERVOMIN);
  sdata.servo /= pulselength;
  pwm.setPWM(0, 0, sdata.servo); 
  // Motor
  sdata.motor = map(sdata.motor, -100, 100, MOTORMAX, MOTORMIN);
  sdata.motor /= pulselength;
  pwm.setPWM(1, 0, sdata.motor);
}



