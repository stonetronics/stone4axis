#include "MotionProcessor.h"
#include "math.h"
#include "pindef.h"

//#define DEBUG
//#define VERBOSE

#define MAX_SPEED   30.0 //mm/s
#define MIN_SPEED   1.0  //mm/s

#define STEPRATE    400 //steps per mm

// returns angle of dy/dx as a value from 0...2PI
static float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


MotionProcessor* MotionProcessor::instance;

MotionProcessor* MotionProcessor::getInstance()
{
  if (instance == NULL)
  {
    instance = new MotionProcessor();
  }
  return instance;
}

MotionProcessor::MotionProcessor( void )
{
  Timer1.initialize();
  enablePin = PIN_EN;
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH); //keep disabled in de beninging
  //init motor pins
  this->motors[0].init(PIN_XDIR, PIN_XSTEP, PIN_XSTOP);  //x
  this->motors[1].init(PIN_YDIR, PIN_YSTEP, PIN_YSTOP);  //y
  this->motors[2].init(PIN_UDIR, PIN_USTEP, PIN_USTOP);  //u
  this->motors[3].init(PIN_VDIR, PIN_VSTEP, PIN_VSTOP);  //v
}

void MotionProcessor::dumbLine(FloatVector coords)
{
  uint8_t i;
  long j;

  uint8_t longest;
  int8_t dir[4];
  long delta[4];

  delta[0] = coords.x * STEPRATE;
  delta[1] = coords.y * STEPRATE;
  delta[2] = coords.u * STEPRATE;
  delta[3] = coords.v * STEPRATE;


  //determine direction and rectify the delta
  for (i = 0; i < 4; i++)
  {
    if (delta[i] < 0)
      dir[i] = -1;
    else
      dir[i] = 1;

    delta[i] = abs(delta[i]);
  }

  //determine longest(=fastest) axis
  longest = delta[0];
  for (i = 1; i < 4; i++)
  {
    if (longest < delta[i])
      longest = delta[i];
  }

  //step all through
  for (j = 0; j < longest; j++)
  {
    for(i = 0; i < 4; i++)
    {
      if (j < delta[i])
      //Serial.println("s");
      pause(linearStepDelay);
      {
        motors[i].stepOnce(dir[i]);
      }
    }
    pause(linearStepDelay);
  }

}

void MotionProcessor::line(FloatVector coords)
{
  uint8_t i;

  long stepDelay;

  delta[0] = coords.x * STEPRATE;
  delta[1] = coords.y * STEPRATE;
  delta[2] = coords.u * STEPRATE;
  delta[3] = coords.v * STEPRATE;

  //in absolute mode, take old position into account
  if (mode == ABS)
  {
    delta[0] -= currentPositionSteps.x;
    delta[1] -= currentPositionSteps.y;
    delta[2] -= currentPositionSteps.u;
    delta[3] -= currentPositionSteps.v;
  }


  #if defined(VERBOSE) || defined(DEBUG)
  Serial.println("line command:");
  Serial.print("delta x: ");
  Serial.println(delta[0]);
  Serial.print("delta y: ");
  Serial.println(delta[1]);
  Serial.print("delta u: ");
  Serial.println(delta[2]);
  Serial.print("delta v: ");
  Serial.println(delta[3]);
  #endif

  //determine the "fastest" axis (biggest delta)
  fastest = 0;
  for (i = 1; i < 4; i++)
  {
    if (abs(delta[fastest]) <= abs(delta[i]))
      fastest = i;
  }

  #ifdef DEBUG
  Serial.print("Fastest axis: ");
  Serial.print(fastest);
  #endif

  //set directions for the motors to move
  //initialize error
  for (i = 0; i < 4; i++)
  {
    if (delta[i] < 0)
      dir[i] = -1;
    else
      dir[i] = 1;

    error[i] = abs(delta[fastest])/2;
  }

  #ifdef DEBUG
  Serial.print("error x: ");
  Serial.println(error[0]);
  Serial.print("error y: ");
  Serial.println(error[1]);
  Serial.print("error u: ");
  Serial.println(error[2]);
  Serial.print("error v: ");
  Serial.println(error[3]);
  #endif

  //calculate stepping speed/ delay - compensate for bresenham
  //4 dim: stepDelay = ((float)(linearStepDelay)) * (sqrt(((float)(delta[0]))*((float)(delta[0]))+((float)(delta[1]))*((float)(delta[1]))+((float)(delta[2]))*((float)(delta[2]))+((float)(delta[3]))*((float)(delta[3]))-((float)(delta[fastest]))*((float)(delta[fastest])))) /  ((float)(abs(delta[fastest])));
  //none: stepDelay = (long)linearStepDelay;
  //compensate 2dimensional on the faster side
  if (fastest < 2)
  {
    //calculate with axis 0 & 1 (X & Y)
    stepDelay = ((float)(linearStepDelay)) * (sqrt(((float)(delta[0]))*((float)(delta[0]))+((float)(delta[1]))*((float)(delta[1]))) / ((float)(abs(delta[fastest]))));
  } else {
    //caluclate with axis 2 & 3 (U & V)
    stepDelay = ((float)(linearStepDelay)) * (sqrt(((float)(delta[2]))*((float)(delta[2]))+((float)(delta[3]))*((float)(delta[3]))) / ((float)(abs(delta[fastest]))));
  }

  #ifdef DEBUG
  Serial.print("linear step delay: ");
  Serial.println(linearStepDelay);

  Serial.print("step delay: ");
  Serial.println(stepDelay);
  #endif

  //travel the line, do the bresenham

  //start bresenham in the interrupt
  ready_ = 0;
  iteration = 0;
  Timer1.initialize(stepDelay);
  Timer1.attachInterrupt(bresenham);
}

static void MotionProcessor::bresenham( void )
{
  if ((*instance).iteration < abs((*instance).delta[(*instance).fastest]))
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      if (i == (*instance).fastest) // skip the fastest axis
        continue;

      (*instance).error[i] -= abs((*instance).delta[i]); // calculate error

      if ((*instance).error[i] < 0) //step the motor if the error goes below zero and "refill"
      {
        (*instance).motors[i].stepOnce((*instance).dir[i]);
        (*instance).error[i] += abs((*instance).delta[(*instance).fastest]);
      }
    }
    (*instance).motors[(*instance).fastest].stepOnce((*instance).dir[(*instance).fastest]); //step the fastest axis anyway

    (*instance).iteration++;
    //Serial.println("iterationo");
  } else {
    //detach interrupt
    Timer1.detachInterrupt();
    //Serial.println("fertige");
    (*instance).ready_ = 1;

    //update position
    (*instance).currentPositionSteps.x += (*instance).delta[0];
    (*instance).currentPositionSteps.y += (*instance).delta[1];
    (*instance).currentPositionSteps.u += (*instance).delta[2];
    (*instance).currentPositionSteps.v += (*instance).delta[3];

    //also update mm position
    (*instance).currentPosition.x = (float)(*instance).currentPositionSteps.x / STEPRATE;
    (*instance).currentPosition.y = (float)(*instance).currentPositionSteps.y / STEPRATE;
    (*instance).currentPosition.u = (float)(*instance).currentPositionSteps.u / STEPRATE;
    (*instance).currentPosition.v = (float)(*instance).currentPositionSteps.v / STEPRATE;

  }
}

void MotionProcessor::home()
{
  //enable Motors
  enableMotors();

  //set to normal speed
  setSpeed(5);

  #ifdef VERBOSE
  Serial.println("homing....");
  #endif
  #ifdef DEBUG
  Serial.print("step delay: ");
  Serial.println(linearStepDelay);
  #endif

  //drive into endstops
  while(!(motors[0].endstop() && motors[1].endstop() && motors[2].endstop() && motors[3].endstop()))
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      if (!motors[i].endstop())
        motors[i].stepOnce(-1);
    }
    pause(linearStepDelay);
  }

  //drive one step out
  while(motors[0].endstop()||motors[1].endstop()||motors[2].endstop()||motors[3].endstop())
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      if (motors[i].endstop())
        motors[i].hardStepOnce(1);
    }
    pause(linearStepDelay);
  }

  setPosition(floatZeroVect);
}

void MotionProcessor::registerTryAndExecCallback(void (*tryAndExecCallBack)( void ))
{
  this->tryAndExecCallBack = tryAndExecCallBack;
}

uint8_t MotionProcessor::ready()
{
  //Serial.print("Ready: ");
  //Serial.println(ready_);
  return ready_;
}

void MotionProcessor::setPosition(FloatVector pos)
{
  this->currentPosition = pos;
  this->currentPositionSteps.x = (long)(STEPRATE * pos.x);
  this->currentPositionSteps.y = (long)(STEPRATE * pos.y);
  this->currentPositionSteps.u = (long)(STEPRATE * pos.u);
  this->currentPositionSteps.v = (long)(STEPRATE * pos.v);
}

void MotionProcessor::setSpeed(float speed)
{
  if (speed < MIN_SPEED)
  {
    #ifdef DEBUG
    Serial.print("too little Speed. Setting to ");
    Serial.print(MIN_SPEED);
    Serial.println(" mm/s");
    #endif
    speed = MIN_SPEED;
  } else if (speed > MAX_SPEED) {
    #ifdef DEBUG
    Serial.print("too much Speed. Setting to ");
    Serial.print(MIN_SPEED);
    Serial.println(" mm/s");
    #endif
    speed = MAX_SPEED;
  }

  feedrate = STEPRATE * speed; //steps per second
  linearStepDelay = (long)(1000000.0 / feedrate); //microseconds

  #ifdef DEBUG
  Serial.print("feedrate: ");
  Serial.print(feedrate);
  Serial.println(" steps/s");
  Serial.print("linear step delay: ");
  Serial.print(linearStepDelay);
  Serial.println(" us");
  #endif

}

float MotionProcessor::getSpeed()
{
  return speed;
}

float MotionProcessor::getFeedrate()
{
  return feedrate;
}

FloatVector MotionProcessor::getPosition( void )
{
  return currentPosition;
}

LongVector MotionProcessor::getPositionSteps( void )
{
  return currentPositionSteps;
}

void MotionProcessor::enableMotors( void )
{
  digitalWrite(enablePin, LOW);
}

void MotionProcessor::disableMotors( void )
{
  digitalWrite(enablePin, HIGH);
}

void MotionProcessor::pause(long us)
{
  delay(us/1000);
  delayMicroseconds(us%1000);  // delayMicroseconds doesn't work for values > ~16k.
}

void MotionProcessor::setMode( MoveMode mode )
{
  this->mode = mode;
}

MoveMode MotionProcessor::getMode( void )
{
  return mode;
}
