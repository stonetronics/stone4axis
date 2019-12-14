#include<Arduino.h>
#include"Motor.h"
#include"DataTypes.h"
#include "TimerOne.h"



class MotionProcessor
{
  public:
    static MotionProcessor* getInstance();
    void line(FloatVector l);
    void dumbLine(FloatVector coords);
    void home();
    void setPosition(float x, float y, float u, float v);
    void setPosition(FloatVector pos);
    FloatVector getPosition( void );
    LongVector getPositionSteps( void );
    void setSpeed(float speed);
    float getSpeed();
    float getFeedrate();
    void enableMotors( void );
    void disableMotors( void );
    void setMode( MoveMode mode );
    MoveMode getMode( void );
    void pause(long us);
    void registerTryAndExecCallback( void (*tryAndExecCallBack)( void ));
    uint8_t ready( void ); //returns 1 when ready, 0 when not

    Motor motors[4];

    //do a iteration of the bresenham algorithm
    //is public static to be able to be attached to an interrupt
    static void bresenham( void );

  private:
    //singleton to work around non static methods in interrups
    static MotionProcessor* instance = NULL;
    MotionProcessor();

    void (*tryAndExecCallBack)( void );
    int enablePin;

    float speed; //mm per second
    float feedrate; //steps per second
    long linearStepDelay;
    FloatVector currentPosition;
    LongVector currentPositionSteps;
    MoveMode mode;

    uint8_t  ready_ = 1;

    //bresenham variables
    int8_t dir[4];
    long delta[4];
    long error[4];
    uint8_t fastest;
    long iteration;
    TimerOne* timer;
};
