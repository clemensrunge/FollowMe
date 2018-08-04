#include "Drive.h"

Drive::Drive()
{
  pwm = Adafruit_PWMServoDriver();
  gpsmath = TinyGPS();
  sdata.servo = 0;
  sdata.motor = 0;
  toDest.angle = 0.0f;
  toDest.distance = 0.0f;
  calc_motor = false;  
  calc_position = false;
  eval_compass = false;
  current_pos.lat = 0.0f;
  current_pos.lon = 0.0f;
  destination.lat = 0.0f;
  destination.lon = 0.0f;
  manual_motor = false;
  manual_motor_speed = 0;
}

void Drive::init()
{
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates 
  pwm.setPWM(SYNC, 0, 463); //ca. 2ms SYNC pulse
  write_servo_data();
}


void Drive::set_current_pos(position *current)
{
	current_pos = *current;
	if(destination.lat != 0.0)
	{
		eval_compass = true;
    calc_position = true;
	}
}

void Drive::set_target_pos(position *target)
{
	destination = *target;
	if(current_pos.lat != 0.0)
	{
		eval_compass = true;
    calc_position = true;
	}
}

void Drive::set_vector(vector *to_target)
{
  calc_position = false;
  eval_compass = true;
  toDest = *to_target;
}

void Drive::set_servo(int servo)
{
  eval_compass = false;
  calc_position = false;
  sdata.servo = servo;
}

void Drive::set_motor(int motor)
{
  calc_motor = false;
  sdata.motor = motor;
}

void Drive::update(MechaQMC5883 compass)
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
    int x,y,z;
    float azimuth;
    // Kompass auslesen
    compass.read(&x, &y, &z, &azimuth);
    Serial.print(" C:");
    Serial.println(azimuth);
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
  float pulselength;
  int temp;
  
  // Servo
  temp = int(float(map(sdata.servo, -100, 100, SERVOMAX, SERVOMIN))/PULSELENGTH);
  pwm.setPWM(SERVO, 0, temp);
  // Motor
  temp = int(float(map(sdata.motor, -100, 100, MOTORMAX, MOTORMIN))/PULSELENGTH);
  pwm.setPWM(MOTOR, 0, temp);
}



