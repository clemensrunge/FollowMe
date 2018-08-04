#include "MechaQMC5883.h"
#include <Wire.h>

volatile int qmc_x[N_AVG_DATAPOINTS], qmc_y[N_AVG_DATAPOINTS], qmc_z[N_AVG_DATAPOINTS];
volatile int qmc_x_max, qmc_x_min, qmc_y_max, qmc_y_min;
volatile uint8_t qmc_buffer_index;
volatile bool qmc_buff_ready, qmc_calibration;

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
  qmc_y_min = -32768;
  qmc_y_max = 32767;

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
  qmc_y_min = -32768;
  qmc_y_max = 32767;
  qmc_calibration = true;
}

void MechaQMC5883::stopCalibration()
{
  x_offset = (qmc_x_min+qmc_x_max)/2;
  y_offset = (qmc_y_min+qmc_y_max)/2;
  if(qmc_x_max - x_offset > qmc_y_max - y_offset)
  {
      scale = (float)qmc_x_max / (float)qmc_y_max;
      scale_x = false; //cal.x2 = cal.x; 
  }  
  else
  {
    scale = (float)qmc_y_max / (float)qmc_x_max;
    scale_x = true;
  }
  qmc_calibration = false;
}

void MechaQMC5883::setAngleOffset(int a)
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
  *a = azimuth(x,y);
}

float MechaQMC5883::azimuth(int* a, int* b){
  float azimuth = angle_offset + atan2((int)*a,(int)*b) * 180.0/PI;
  return azimuth < 0?360 + azimuth:azimuth;
  //return azimuth;
}
