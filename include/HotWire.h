#include<Arduino.h>


class HotWire
{
public:
  HotWire(int pin); //constructor

  void on( void );
  void off( void );

private:
  int pin;

};
