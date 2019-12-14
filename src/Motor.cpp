#include "Motor.h"

void Motor::init(int dir, int step, int end)
{
  this->dir = dir;
  this->step = step;
  this->end = end;

  pinMode(this->dir, OUTPUT);
  pinMode(this->step, OUTPUT);
  pinMode(this->end, INPUT_PULLUP);
}

void Motor::stepOnce( int8_t dir )
{
  if (!Motor::endstop())
  {
    Motor::hardStepOnce(dir);
  }
}

void Motor::hardStepOnce( int8_t dir )
{
  if (dir > 0)
    {
      digitalWrite(this->dir, HIGH);
    } else {
      digitalWrite(this->dir, LOW);
    }
    
    digitalWrite(this->step, HIGH);
    digitalWrite(this->step, LOW);
}

uint8_t Motor::endstop( void )
{
  #ifdef DEBUG
  Serial.print("Endstop: ");
  Serial.println(!digitalRead(this->end)); //inverted (0-active)
  #endif
  return !digitalRead(this->end); //inverted (0-active)
}
