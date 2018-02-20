#ifndef BTApp_h
#define BTApp_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#endif

#include "Structs.h"

#define TEST 0
#define GPSTEST 1
#define STOP 2
#define ANGLETEST 3
#define CAMERA 4
#define GPS 5

#define BTDRIVE 100
#define BTSWITCH 99
#define BTSTOP 115
#define BTPOSITION 112

class BTApp
{	
public:
  BTApp();
	BTApp(int inital_mode);
	void new_data(char c);
	int get_mode();
	void get_position(position *pos);
private:
  bool position_send;
  byte index;
  char lon_array[10];
  char lat_array[10];
	byte command;
	byte mode, last_mode;
	position destination;	
};
#endif

