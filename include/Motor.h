#include<Arduino.h>

class Motor{
  public:
    void init(int dir, int step, int end);
    void stepOnce( int8_t dir ); //dir = -1 / 1 
    void hardStepOnce( int8_t dir ); //dir = -1 / 1; ingores endstops
    uint8_t endstop( void ); //returns 1 if endstop is pressed
  private:
    int dir;
    int step;
    int end;
};
