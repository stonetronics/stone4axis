//------------------------------------------------------------------------------
// stone4axisHotWireCutter - 4 axis hotwire cutter gcode interpreter for CNCShieldV3 on arduino UNO
// This project is based on http://www.github.com/MarginallyClever/GcodeCNCDemo, thank you

//the line moving is interrupt driven
//it utilizes a buffer for each input line received (defines for line length and buffersize in LineBuffer.h)
//when executing a line, the next execution is fetched within the last interrupt via callback to main.cpp

// This interpreter understands XYUV gcode
// The hotwire is simple ON/OFF and is controlled by M3-On and M5-Off commands

//example commands:
//G1 X50 Y0 U50 V0
//G1 X0 Y50 U0 V50
//G1 X-50 Y-50 U-50 V-50



#include"DataTypes.h"
#include"MotionProcessor.h"
#include"HotWire.h"
#include"LineBuffer.h"
#include"pindef.h"

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

//#define VERBOSE
//#define ENDSTOPDEBUG              //uncomment for endstop debugging

#define VERSION              (1)        // firmware version
#define MAX_BUF              MAX_LINE_LENGTH
#define BAUD                 (115200)   // How fast is the Arduino talking?(BAUD Rate of Arduino)

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

void processCommand(char* command); //function prototype

char buffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer
long line_number=0;

LineBuffer lineBuffer;
MotionProcessor* motionProcessor = MotionProcessor::getInstance();
HotWire hotwire(PIN_WIRE);

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parseNumber(char* command, char code, float val) {
  char* ptr =command;
  uint8_t i = 0;
  while((ptr != NULL) && (ptr[i]!='\0') && (i<MAX_BUF)) {  // walk to the end
    if(ptr[i++]==code) {  // if you find code on your walk,
      return atof(ptr+i);  // convert the digits that follow into a float and return it
    }
  }
  return val;  // end reached, nothing found, return default val.
}

//read a whole line into a string
uint8_t tryAndGetLine(char* output)
{
  static char buffer[MAX_BUF];
  static int sofar;

  // listen for serial commands
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // echo
    if(sofar<MAX_BUF-1) buffer[sofar++]=c;  // store it
    if(c=='\n') {
      // entire line received
      buffer[sofar]='\0';  // end the buffer so string functions work right

      //copy into buffer
      uint8_t i = 0;
      while (buffer[i]!='\0')
      {
        output[i]=buffer[i];
        i++;
      }
      output[i] = '\0';
      sofar = 0; //reset counter
      return 1;
    }
  }
  return 0;
}

/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(char *code,float val) {
  Serial.print(code);
  Serial.print(val);
  Serial.print(" ");
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  FloatVector pos = motionProcessor->getPosition();
  output("X",pos.x);
  output("Y",pos.y);
  output("U",pos.u);
  output("V",pos.v); output("F",motionProcessor->getFeedrate());
  Serial.println(motionProcessor->getMode()?"ABS":"REL");
}


/**
 * display helpful information
 */
void help() {
  Serial.print(F(" === stone4axis ==="));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00/G01 [X/Y/Z/E(mm)] [F(speed)]; - linear move"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G28; - perform homing routine"));
  Serial.println(F("G90; - buf_[head_] = item;absolute mode | always give all coordinates here, otherwise the missing will drive to zero position"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X/Y/U/V(mm)]; - change logical position"));
  Serial.println(F("M17; - enable motors"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}


/*
check if something is in line buffer and execute if so
*/
void tryAndExecute( void )
{
  if (!lineBuffer.empty())
  {
    char nextCommand[MAX_BUF];
    lineBuffer.getLine(nextCommand);
    processCommand(nextCommand);
  }
}

/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand(char* command) {
  FloatVector parsedVector;

  int cmd = parseNumber(command,'G',-1);
  switch(cmd) {
    case  0:
    case  1:  // command
      motionProcessor->setSpeed(parseNumber(command,'F', 5.0));
      parsedVector.x = parseNumber(command,'X',0);
      parsedVector.y = parseNumber(command,'Y',0);
      parsedVector.u = parseNumber(command,'U',0);
      parsedVector.v = parseNumber(command,'V',0);
      motionProcessor->line( parsedVector );
      break;

    case  2:
    case  4:  motionProcessor->pause(parseNumber(command,'P',0)*1000);  break;  // dwell
    case 28:  motionProcessor->home(); //homing routine
    case 90:  motionProcessor->setMode(ABS);  break;  // absolute mode
    case 91:  motionProcessor->setMode(REL);  break;  // relative mode
    case 92:  // set logical position
      parsedVector.x = parseNumber(command,'X',0);
      parsedVector.y = parseNumber(command,'Y',0);
      parsedVector.u = parseNumber(command,'U',0);
      parsedVector.v = parseNumber(command,'V',0);
      motionProcessor->setPosition( parsedVector );
      break;
    default:  break;
  }

  cmd = parseNumber(command,'M',-1);
  switch(cmd) {
    case   3:  hotwire.on(); break;
    case   5:  hotwire.off(); break;
    case  17:  motionProcessor->enableMotors();  break;
    case  18:  motionProcessor->disableMotors();  break;
    case 100:  help();  break;
    case 101:
      motionProcessor->setSpeed(parseNumber(command,'F', 5.0));
      parsedVector.x = parseNumber(command,'X',0);
      parsedVector.y = parseNumber(command,'Y',0);
      parsedVector.u = parseNumber(command,'U',0);
      parsedVector.v = parseNumber(command,'V',0);
      motionProcessor->dumbLine( parsedVector );
      break;
    case 114:  where();  break;
    default:  break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}


#ifdef ENDSTOPDEBUG
long prevMillis;
#endif

void setup() {

  Serial.begin(BAUD);  // open coms

  #ifdef VERBOSE
  where();
  help();  // say hello
  #endif

  motionProcessor->registerTryAndExecCallback(tryAndExecute); //register try and execute to be executed right after a line finishes
  motionProcessor->setPosition(floatZeroVect);  // set starting position
  motionProcessor->setSpeed(5.0);  // set default speed mm/s


  ready();
}

void loop() {

  //input buffer
  char buf[MAX_BUF];

  #ifdef ENDSTOPDEBUG
  if ((millis() - prevMillis) > 100) {
    prevMillis = millis();
    //debug: output endstops
    Serial.print("x: ");
    Serial.print(motionProcessor->motors[0].endstop());
    Serial.print(" | y: ");
    Serial.print(motionProcessor->motors[1].endstop());
    Serial.print(" | u: ");
    Serial.print(motionProcessor->motors[2].endstop());
    Serial.print(" | v: ");
    Serial.println(motionProcessor->motors[3].endstop());
  }
  #endif

  //produzzer
  if (!lineBuffer.full()) //if we got place to receive
  {
    if (tryAndGetLine(buf)) //receive
    {
      lineBuffer.putLine(buf);
      ready();
    }
  }

  //konzzummer
  if (!lineBuffer.empty())
  {
    if (motionProcessor->ready())
    {
      lineBuffer.getLine(buf);
      processCommand(buf);
    }
  }




}
