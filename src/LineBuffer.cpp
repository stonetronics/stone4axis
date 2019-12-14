#include"LineBuffer.h"

void LineBuffer::putLine(char* line)
{
  if (full_)
  {
    return;
  }

  //copy line into buffer
  uint8_t i = 0;
  while((line[i]!='\0')&&(i < MAX_LINE_LENGTH-1))
  {
    buffer[head][i] = line[i];
    i++;
  }
  buffer[head][i] = '\0';

	head = (head + 1) % BUFFERSIZE;

	full_ = head == tail;
}

void LineBuffer::getLine(char* output)
{
  if(empty())
	{
    output[0] = '\0';
		return;
	}

	//Read data advance the tail (we now have a free space)
	uint8_t i = 0;
  while((buffer[tail][i] != '\0')&&(i < MAX_LINE_LENGTH-1))
  {
    output[i] = buffer[tail][i];
    i++;
  }
  output[i] = '\0';
  //advance the tail (we now have a free space)
	full_ = 0;
	tail = (tail + 1) % BUFFERSIZE;

	return;
}

void LineBuffer::reset( void )
{
  head = tail;
  full_ = 0;
}

uint8_t LineBuffer::empty( void )
{
	return (!full_ && (head == tail));
}

uint8_t LineBuffer::full( void )
{
  return full_;
}

uint8_t LineBuffer:: size()
{
	uint8_t size = BUFFERSIZE;

	if(!full_)
	{
		if(head >= tail)
		{
			size = head - tail;
		}
		else
		{
			size = BUFFERSIZE + head - tail;
		}
	}

	return size;
}
