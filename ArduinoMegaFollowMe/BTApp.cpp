#include "BTApp.h"
BTApp::BTApp()
{
  
}

BTApp::BTApp(int inital_mode)
{
	command = 0;
  mode = STOP;
  last_mode = inital_mode;
}

void BTApp::new_data(char data_byte)
{
  if (false == position_send)
  {
    switch(data_byte)
    {
      case BTPOSITION: 
        position_send = true; 
        index = 0;
        break;
      case BTDRIVE:
        mode = last_mode; 
        break;
      case BTSTOP: 
        last_mode = mode;
        mode = STOP; 
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
}

int BTApp::get_mode()
{
  return mode;
}

void BTApp::get_position(position *pos)
{
	*pos = destination;
}
