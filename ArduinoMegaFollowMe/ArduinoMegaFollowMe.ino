#include "BTApp.h"
#include "Drive.h"
#include "Raspi.h"
#include "Servo_read.h"
#include <TinyGPS.h>

void setup();
void loop();
void servo_sync();
void asynchron_tasks();
void test_mode();
void gps_test_mode();
void angle_test_mode();
void camera_mode();
void gps_mode();

#define SERVO_SYNC_IN A1
#define SERVO_SYNC_ERROR A2
#define BTRX 10
#define BTTX 11
#define RASPIRX 6
#define RASPITX 7

BTApp btapp;
Drive drive;
Raspi raspi;
Servo_read servo_read;
bool mainloop_sync;
TinyGPS gps;

void setup() {
  btapp = BTApp(TEST);
  drive = Drive();
  raspi = Raspi();
  servo_read = Servo_read();
  TinyGPS gps = TinyGPS();
  Serial.begin(115200, SERIAL_8N1);  //HOST PC
  Serial1.begin(115200, SERIAL_8N1); //BT Module
  Serial2.begin(9600, SERIAL_8N1);   //GPS Module
  Serial3.begin(115200, SERIAL_8N1); //Raspi
  Serial.println("[INFO] setup done")
}

void loop() {
  switch(btapp.get_mode()){
    case STOP: stop_mode(); break;
    case TEST: test_mode(); break;
    case GPSTEST: gps_test_mode(); break;
    case ANGLETEST: angle_test_mode(); break;
    case CAMERA: camera_mode(); break;
    case GPS: gps_mode(); break;
  }
}

void servo_sync()
{
  static bool mainloop_sync = false;
  unsigned long time_start = millis();  
  while(digitalRead(SERVO_SYNC_IN)) 
  {
    mainloop_sync = true;
  }
  if(mainloop_sync)
  {
    mainloop_sync = false;
    return;
  }
  else
  {
    asynchron_tasks();
    if(30 > time_start - millis())
    {     
      Serial.print("[ERROR] No Servo Sync Pulse on Pin ");
      Serial.print(SERVO_SYNC_IN);
      Serial.println(".");
      delay(1000);
    }
  }
}

void asynchron_tasks()
{
  if(Serial1.available())
  {
    btapp.new_data(Serial1.read());
  }
}

void stop_mode()
{
  Serial.println("[INFO] starting mode: STOP");
  while(STOP == btapp.get_mode())
  {
    drive.set_motor(0);
    asynchron_tasks();
  }
}

void test_mode()
{
  servo_data newServoData;
  Serial.println("[INFO] starting mode: TEST");
  while(TEST == btapp.get_mode())
  {
    servo_sync();    
    servo_read.get_servo_data(&newServoData);
    Serial.print("M:");
    Serial.print(newServoData.motor);
    Serial.print(" S:");
    Serial.println(newServoData.servo);
    drive.set_servo(newServoData.servo);
    drive.set_motor(newServoData.motor);
      
    /*
    int pulselen;
      setServoPulse(SERVO, 0);
      Serial.println(" Ausschlag von -100 (rückwärts) bis 100 (vorwärts)");
      while(Serial.available() == 0); // Wartet auf eingegebenen Wert, da parseInt Timeout hat
      pulselen = Serial.parseInt();
      setServoPulse(MOTOR, pulselen);
      Serial.println(pulselen);
      */
  }
}

void angle_test_mode()
{
  while(ANGLETEST == btapp.get_mode())
  {
    servo_sync();
    /*
     * int CompassAngle; 
      setServoPulse(MOTOR, MotorRecieved); 
      CompassAngle = getCompassAngle();
      driveAngle(79, CompassAngle);
      break;
     */
  }
}
void gps_test_mode()
{
  
  while(GPSTEST == btapp.get_mode())
  {
    servo_sync();
    /*
     * 
     *      bool newData = false;
      // For one second we parse GPS data and report some key values
      for (unsigned long start = millis(); millis() - start < 1000;){
        while (ss.available()){
          char c = ss.read();
          //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
          if (gps.encode(c)) // Did a new valid sentence come in?
            newData = true;
        }
      }
      if (newData){
        static struct position destination;
        static struct position car;
        float flat, flon, GPSangle, distance;
        unsigned long age;
        static struct position lastPosition;
        gps.f_get_position(&car.lat, &car.lon, &age);
        GPSangle = gps.course_to(car.lat, car.lon, destination.lat, destination.lon);
        distance = gps.distance_between(car.lat, car.lon, destination.lat, destination.lon);
        Serial.print("lat: ");
        Serial.print(car.lat, 6);
        Serial.print(", lon: ");
        Serial.println(car.lon);
        Serial.print("Distance: ");
        Serial.println(distance);
        Serial.print("Angle: ");
        Serial.println(GPSangle);
        Serial.print("Compass Angle: ");
        driveAngle(GPSangle, getCompassAngle());
        driveDistance(distance);
        }
      
     */
  }
}

void camera_mode()
{
  while(CAMERA == btapp.get_mode())
  {
    servo_sync();

  }
}

void gps_mode()
{
  while(GPS == btapp.get_mode())
  {
    servo_sync();
    /*
     * Serial.println("Appmodus");
        String data; //GPS Daten der App
        unsigned long age;
        bool newData;
        newData = false;
        static struct position destination;
        static struct position car;
        static struct vector course;
        unsigned long start;
        
        // For one second we parse GPS data and report some key values
        for (start = millis(); millis() - start < 1000;){
        while (ss.available()) {
          char c = ss.read();
          //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
          if (gps.encode(c)) // Did a new valid sentence come in?
            newData = true;
        }
        if (newData){
        gps.f_get_position(&car.lat, &car.lon, &age);
        course.angle = gps.course_to(car.lat, car.lon, destination.lat, destination.lon);
        course.distance = gps.distance_between(car.lat, car.lon, destination.lat, destination.lon);
        }
        }
        // Auslesen der Daten Bluetooth
        if (btSerial.available()) {
          data = btSerial.readString();
          Serial.println(data);
          
          if (data == "switch") {}    
        else if (data == "drive") {
          Serial.println("Drive");
          driveAngle(course.angle, getCompassAngle());
          driveDistance(course.distance);
          }
          else if (data == "stop") {
            Serial.println("stop");
            setServoPulse(MOTOR, 0);
            setServoPulse(SERVO, 0);
          }else {
            destination.lon = getLongitude(data);
            destination.lat = getLatitude(data);
          }
        }
        }
          break;
        
          default: Serial.println("Default"); 
          break;
      }
    } 
  
     */
  }
}
