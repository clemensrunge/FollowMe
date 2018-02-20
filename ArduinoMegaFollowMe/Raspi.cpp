#include "Raspi.h" 

Raspi::Raspi()
  {
    toPerson.angle = 0.0f;
    toPerson.distance = 0.0f;
    current_command = 0;
    in_command_transfer = false;
  }
void Raspi::new_data(char c)
 {
  if (false == in_command_transfer)
  {
    current_command = c;
    switch(c)
    {
      case RASPIDISTANCE:
        command_index = 0;        
        break;
      case PASPIANGLE:
        command_index = 0;
        break;
      default:
        current_command = 0;
        break;
    }
  }
  else
  {
    switch(c)
    {
      case RASPIDISTANCE:
        command_index = 0;
        break;
      case PASPIANGLE:
        command_index = 0;
        break;
    }
    
  }

}
void  Raspi::get_vector(vector *v)
{
  
}

