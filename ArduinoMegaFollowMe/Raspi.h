#ifndef Raspi_h
#define Raspi_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#endif
#include "Structs.h"

#define RASPIDISTANCE 'D'
#define PASPIANGLE 'A'

class Raspi
{
public:
  Raspi();
  void new_data(char c);
  void get_vector(vector *v);
private:
  vector toPerson;
  byte command_index, current_command;
  bool in_command_transfer;
};
#endif
