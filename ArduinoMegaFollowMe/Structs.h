#ifndef Structs_h
#define Structs_h

struct position{ 
  float lat;
  float lon;
};

struct vector{ 
  float angle;
  float distance;
};

struct servo_data{
	int servo;
	int motor;
};
#endif