#ifndef Structs_h
#define Structs_h

typedef struct{ 
  float lat;
  float lon;
}position;

typedef struct { 
  float angle;
  float distance;
}vector;

typedef struct {
	int servo;
	int motor;
}servo_data;
#endif
