#ifndef Controller_h
#define Controller_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#endif

#include "Structs.h"

#define BUFFERSIZE_COMMANDS 10
#define BUFFERSIZE_SENSOR_REQUESTS 10

enum MODES {
  M_RC_CAR,
  M_STRAIGHT,
  M_ANGLE,
  M_CURVE,
  M_ANGLE_BY_REMOTE
};

const uint8_t ARGUMENTS_PER_MODE[] {
  0,
  1, //speed
  2, //speed, relative angle
  2, //speed, absolute angle
  1  //inital angle  
};

enum COMMANDS {
  C_START,
  C_STOP,
  C_ENABLE_AUTONOMOUS_MOTOR_OUT, 
  C_DISABLE_AUTONOMOUS_MOTOR_OUT, //Motor Output is taken straight from the Remote
  C_ENABLE_BTAPP_TO_SWITCH_MODE,
  C_DISABLE_BTAPP_TO_SWITCH_MODE,
  C_START_MAG_CAL,
  C_FINISH_MAG_CAL,
  C_SET_MAG_NORTH,  
};

enum SENSORS {
  S_HEADING,
  S_REMOTE_DRIVE,
  S_REMOTE_STEERING,  //Values from the Remote Control
  S_LATITUDE,
  S_LONGITUDE,
  S_TARGET_LATITUDE,
  S_TARGET_LONGITUDE,
  S_BTAPP_MODE,
};

enum UART_CODES {
  UART_MODE = 'M',
  UART_COMMAND = 'C',
  UART_SENSOR = 'S'
};

class Controller
{	
public:
	Controller(int inital_mode);
	void new_data(char c);
	int get_mode();
  void set_mode(int m, int *arguments);
  void set_command(int c);
  void set_sensor_request(int s);
	bool has_commands();
	int get_next_command();
	bool has_sensor_request();
	int get_next_sensor_request();
private:
	void parse_mode(char c);
	void parse_command(char c);
	void parse_sensor_request(char c);
  	bool send_mode,
  		 send_command,
  		 send_sensor;
  	int recieve_index, mode;
    String recieve_buffer;
    int mode_arguments[10];
    int mode_argument_parse_pointer;
  	int commands[BUFFERSIZE_COMMANDS];
  	int sensor_requests[BUFFERSIZE_SENSOR_REQUESTS];
  	int *command_ptr_top;
    int *command_ptr_bot;
  	int *sensor_requests_ptr_top;
   int *sensor_requests_ptr_bot;
};
#endif

