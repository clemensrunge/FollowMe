#include "Controller.h"

Controller::Controller(int inital_mode)
{
  mode = inital_mode;
  send_mode = false;
  send_command = false;
  send_sensor = false;
  command_ptr_top = commands;
  command_ptr_bot = commands;
  sensor_requests_ptr_top = sensor_requests;
  sensor_requests_ptr_bot = sensor_requests;
  mode_argument_parse_pointer = mode_arguments[0];
}

void Controller::new_data(char data_byte)
{
  if (true == send_mode) parse_mode(data_byte);
  else if (true == send_command) parse_command(data_byte);
  else if(true == send_sensor) parse_sensor_request(data_byte);
  else
  {
    recieve_index = 0;
    recieve_buffer = "";
    switch(data_byte)
    {
      case UART_MODE:
        send_mode = true;
        break;
      case UART_COMMAND:
        send_command = true;
        break;
      case UART_SENSOR:
        send_sensor = true;
        break;
    }
  }  
}

void Controller::set_mode(int m, int *arguments)
{
  if(m <= M_ANGLE_BY_REMOTE && m >= 0)
  {
    int i;
    for(i = 0; i < ARGUMENTS_PER_MODE[m]; i++)
    {
      mode_arguments[i] = arguments[i];
    }
    mode = m;    
  }
}
void Controller::set_command(int c)
{
  if(c <= C_SET_MAG_NORTH && c >= 0)
  {
    command_ptr_top++;
    if(command_ptr_top > commands[BUFFERSIZE_COMMANDS -1])
    {
      command_ptr_top = commands[0];
    }
    *command_ptr_top = c;
  }
}
void Controller::set_sensor_request(int s)
{
  if(s <= S_BTAPP_MODE && s >= 0)
  {
    sensor_requests_ptr_top++;
    if(sensor_requests_ptr_top > sensor_requests[BUFFERSIZE_SENSOR_REQUESTS -1])
    {
      *sensor_requests_ptr_top = sensor_requests[0];
    }
    *sensor_requests_ptr_top = s;
  }
}

void Controller::parse_mode(char c)
{
  recieve_buffer += c;
  
  if(c == '\n' || c == '\r')
  {
    int next_mode = recieve_buffer.toInt();
    set_mode(next_mode, mode_arguments);
    recieve_index = 0;
    mode_argument_parse_pointer = mode_arguments[0];
    send_mode = false;
  }
  else if( c == " ")
  {
    mode_argument_parse_pointer = recieve_buffer.toInt();
    mode_argument_parse_pointer++;
    recieve_buffer = "";
  }
};
void Controller::parse_command(char c)
{
  recieve_buffer[recieve_index] = c;
  if(c == '\n' || c == '\r')
  {
    int next_command = recieve_buffer.toInt();
    set_command(next_command);
    recieve_index = 0;
    send_command = false;
  }
};
void Controller::parse_sensor_request(char c)
{
  recieve_buffer[recieve_index] = c;
  if(c == '\n' || c == '\r')
  {
    int next_sensor = recieve_buffer.toInt();
    set_sensor_request(next_sensor);    
    recieve_index = 0;
    send_sensor = false;
  }
};

int Controller::get_mode()
{
  return mode;
}

bool Controller::has_commands()
{
  return command_ptr_top != command_ptr_bot;
};

int Controller::get_next_command()
{
  int r = *command_ptr_bot;
  command_ptr_bot++;
  if(command_ptr_bot > commands[BUFFERSIZE_COMMANDS -1])
  {
    command_ptr_bot = commands[0];
  }
  return r;
};
bool Controller::has_sensor_request()
{
  return sensor_requests_ptr_top != sensor_requests_ptr_bot;
};
int Controller::get_next_sensor_request()
{
  int r = *sensor_requests_ptr_bot;
  sensor_requests_ptr_bot++;
  if(sensor_requests_ptr_bot > sensor_requests[BUFFERSIZE_SENSOR_REQUESTS -1])
  {
    sensor_requests_ptr_bot = commands[0];
  }
  return r;
};
