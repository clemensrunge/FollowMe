#include "BTApp.h"
BTApp::BTApp()
{
  position_send = false;
  mode_send = false;
  command = 0;
  mode = STOP;
  last_mode = STOP;;
}

BTApp::BTApp(int inital_mode)
{
	command = 0;
  mode = STOP;
  last_mode = inital_mode;
  position_send = false;
  mode_send = false;
  
}

void BTApp::new_data(char data_byte)
{
  if (false == position_send && false == mode_send)
  {
    switch(data_byte)
    {
      case BTPOSITION: 
        position_send = true;
        index = 0;
        break;
      case BTDRIVE:
        if(last_mode != mode)
        {
          mode = last_mode;
        }
        break;
      case BTSTOP:
        if(STOP != mode)
        {
          last_mode = mode;
          mode = STOP; 
        }
        break;
      case MODESWITCH:
        mode_send = true;
        break;
      case BTSWITCH:
        if( mode == GPS)
        {
          mode = CAMERA;
        }
        else if(mode == CAMERA)
        {
          mode = GPS;
        }
        break;
    }
  }
  else
  {
    if(position_send)
    {
      if (data_byte != 13 && data_byte != 10) 
      {
        if(index < 9) {
          lon_array[index] = data_byte;
          index++;
        } 
        else 
        {
          lat_array[index-9] = data_byte;
          index++;
        }
      }
      if (index == 18)
      {
        position_send = false;
        destination.lon = String(lon_array).toFloat();
        destination.lat = String(lat_array).toFloat();
      }
    }
    if(mode_send)
    {
      mode_send = false;
      bool test = false;
      data_byte -= 48;
      test = (data_byte == TEST)||
             (data_byte == GPSTEST)||
             (data_byte == STOP)||
             (data_byte == ANGLETEST)||
             (data_byte == CAMERA)||
             (data_byte == GPS);
      if(test)
      {
        mode = data_byte;
      }
    }
  }  
}

int BTApp::get_mode()
{
  return mode;
}

void BTApp::set_mode(byte m)
{
  mode = m;
}

void BTApp::get_position(position *pos)
{
	*pos = destination;
}
