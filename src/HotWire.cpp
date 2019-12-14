#include "HotWire.h"

//#define VERBOSE


HotWire::HotWire(int pin)
{
  this->pin = pin;
  pinMode(this->pin, OUTPUT);
  digitalWrite(this->pin, LOW); //turn off in the beninging
}

void HotWire::on( void )
{
  digitalWrite(this->pin, HIGH);
  #ifdef VERBOSE
  Serial.println("turning on Hotwire");
  #endif
}

void HotWire::off( void )
{
  digitalWrite(this->pin, LOW);
  #ifdef VERBOSE
  Serial.println("turning off Hotwire");
  #endif
}
