
#ifndef __DATA_TYPES_H__
#define __DATA_TYPES_H__

typedef struct {
  long x;
  long y;
  long u;
  long v;
} LongVector;

typedef struct {
  float x;
  float y;
  float u;
  float v;
} FloatVector;

const FloatVector floatZeroVect = { 0.0, 0.0, 0.0, 0.0 };

/*
typedef struct {
  FloatVector from;
  FloatVector to;
} Line;

typedef struct {
  LongVector from;
  LongVector to;
} LongLine; */

typedef enum{
  ABS = 1,
  REL = 0
} MoveMode;

#endif
