
#ifndef __PINDEF_H__
#define __PINDEF_H__

//connections on the cnc CNCShieldV3

#define PIN_ABORT   A0
#define PIN_HOLD    A1
#define PIN_RESUME  A2
#define PIN_COOLANT A3
#define PIN_XSTEP   2
#define PIN_YSTEP   3
#define PIN_ZSTEP   4
#define PIN_XDIR    5
#define PIN_YDIR    6
#define PIN_ZDIR    7
#define PIN_EN      8
#define PIN_XSTOP   9
#define PIN_YSTOP   10
#define PIN_ZSTOP   11
#define PIN_SPINEN  12
#define PIN_SPINDIR 13

//defines for better readability

#define PIN_USTEP   PIN_ZSTEP
#define PIN_UDIR    PIN_ZDIR
#define PIN_USTOP   PIN_ZSTOP
#define PIN_VSTEP   PIN_SPINEN
#define PIN_VDIR    PIN_SPINDIR
#define PIN_VSTOP   PIN_COOLANT

#define PIN_WIRE    PIN_HOLD

#endif
