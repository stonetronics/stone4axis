#ifndef __LINEBUFFER_H__
#define __LINEBUFFER_H__

#include<Arduino.h>

#define MAX_LINE_LENGTH   64
#define BUFFERSIZE  8

class LineBuffer
{
  public:
    LineBuffer(){};
    void putLine(char* line); //put a line as a string, terminated with a '\0'
    void getLine(char* output);
    void reset( void );
    uint8_t empty( void );
    uint8_t full( void );
    uint8_t size( void );
  private:

    //buffer
    char buffer[BUFFERSIZE][MAX_LINE_LENGTH];

    uint8_t head = 0;
    uint8_t tail = 0;
    uint8_t full_ = 0;
};

#endif
