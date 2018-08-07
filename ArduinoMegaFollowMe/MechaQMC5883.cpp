#include "MechaQMC5883.h"
#include <Wire.h>
boolean MechaQMC5883::available()
{
  return digitalRead(NEW_DATA_INT_PIN);
}

void MechaQMC5883::newData()
{
  Wire.beginTransmission(QMC5883_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(QMC5883_ADDR, 6);
  qmc_x[qmc_buffer_index] = Wire.read(); //LSB  x
  qmc_x[qmc_buffer_index] |= Wire.read() << 8; //MSB  x
  qmc_y[qmc_buffer_index] = Wire.read(); //LSB  z
  qmc_y[qmc_buffer_index] |= Wire.read() << 8; //MSB z
  qmc_z[qmc_buffer_index] = Wire.read(); //LSB y
  qmc_z[qmc_buffer_index] |= Wire.read() << 8; //MSB y
  
  if(true == qmc_calibration)
  {
    if(qmc_x[qmc_buffer_index] > qmc_x_max) qmc_x_max = qmc_x[qmc_buffer_index];
    if(qmc_x[qmc_buffer_index] < qmc_x_min) qmc_x_min = qmc_x[qmc_buffer_index];
    if(qmc_y[qmc_buffer_index] > qmc_y_max) qmc_y_max = qmc_y[qmc_buffer_index];
    if(qmc_y[qmc_buffer_index] < qmc_y_min) qmc_y_min = qmc_y[qmc_buffer_index];
  }

  //Serial.print("comp: ");
  //Serial.println(qmc_x[qmc_buffer_index]);
  qmc_buffer_index++;
  if(N_AVG_DATAPOINTS == qmc_buffer_index + 1)
  {
    qmc_buffer_index = 0;
    qmc_buff_ready = true;
  }  
}

void MechaQMC5883::WriteReg(byte Reg,byte val){
  Wire.beginTransmission(QMC5883_ADDR); //start talking
  Wire.write(Reg); // Tell the HMC5883 to Continuously Measure
  Wire.write(val); // Set the Register
  Wire.endTransmission();
}

void MechaQMC5883::init(){
  WriteReg(0x0B,0x01);
  //Define Set/Reset period
  setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);
  /*
  Define
  OSR = 512
  Full Scale Range = 8G(Gauss)
  ODR = 200HZ
  set continuous measurement mode
  */
  qmc_buffer_index = 0;
  qmc_calibration = false;
  qmc_buff_ready = false;
  x_offset = 0;
  y_offset = 0;
  angle_offset = 0;
  qmc_x_max = -32768;
  qmc_x_min = 32767;
  qmc_y_max = -32768;
  qmc_y_min = 32767;

  pinMode(NEW_DATA_INT_PIN, INPUT);
  digitalWrite(NEW_DATA_INT_PIN, LOW); 
  //attachInterrupt(digitalPinToInterrupt(NEW_DATA_INT_PIN), ISR_QMC5883_New_Data, RISING);
}

void MechaQMC5883::setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr){
  WriteReg(0x09,mode|odr|rng|osr);
}

void MechaQMC5883::softReset(){
  WriteReg(0x0A,0x80);
}

void MechaQMC5883::startCalibration()
{
  qmc_x_max = -32768;
  qmc_x_min = 32767;
  qmc_y_max = -32768;
  qmc_y_min = 32767;
  qmc_calibration = true;
}

void MechaQMC5883::stopCalibration()
{
  int x,y;
  x_offset = (qmc_x_min+qmc_x_max)/2;
  y_offset = (qmc_y_min+qmc_y_max)/2;
  x = qmc_x_max - x_offset;
  y = qmc_y_max - y_offset;
  if( x > y)
  {
      scale = (float)x / (float)y;
      scale_x = false; //cal.x2 = cal.x; 
  }  
  else
  {
    scale = (float)y / (float)x;
    scale_x = true;
  }
  qmc_calibration = false;
  Serial.print("Calibration End\nxoffset: ");
  Serial.println(x_offset);
  Serial.print("yoffset: ");
  Serial.println(y_offset);
  Serial.print("scale: ");
  Serial.println(scale);
  Serial.print("x_min: ");
  Serial.println(qmc_x_min);
  Serial.print("x_max: ");
  Serial.println(qmc_x_max);
  Serial.print("y_min: ");
  Serial.println(qmc_y_min);
  Serial.print("y_max: ");
  Serial.println(qmc_y_max);
  
}

void MechaQMC5883::setNorth()
{
  int x,y,z;
  float azimuth;
  read(&x, &y, &z, &azimuth);
  setAngleOffset(azimuth);
}

void MechaQMC5883::setAngleOffset(float a)
{
  angle_offset = a;
}

void MechaQMC5883::read(int* x,int* y,int* z,float* a){
  int8_t n,i = qmc_buffer_index -1;
  float x_avg = 0, y_avg = 0;
  for(n = 0; n < N_AVG_DATAPOINTS; n++)
  {
    if(0 > i)
    {
      i = N_AVG_DATAPOINTS-1;
    }
    x_avg += qmc_x[i];
    y_avg += qmc_y[i];
    i--;
  }
  
  *x = (int)(x_avg/N_AVG_DATAPOINTS - x_offset);
  *y = (int)(y_avg/N_AVG_DATAPOINTS - y_offset);

  if(true == scale_x)
  {
    *x = (int)((float)*x * scale);
  }
  else
  {
    *y = (int)((float)*y * scale);
  }
  *a = azimuth(x,y); //360-x --> invert
}

float MechaQMC5883::azimuth(int* a, int* b){
  float azimuth = atan2((int)*a,-(int)*b) * 180.0/PI - angle_offset;
  return azimuth < 0?360 + azimuth:azimuth;
  //return azimuth;
}
