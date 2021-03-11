# stone4axis
4 axis gcode interpreter for controlling a CNC hotwire.

This project is based on http://www.github.com/MarginallyClever/GcodeCNCDemo, thank you

This repository contains firmware for a 4 axis cnc hotwire machine controlled by an arduino uno and a CNC SHIELD. Atom with PlatformIO was used instead of Arduino IDE to have better overview.

## hardware

### hotwire machine

X and Y axis are on the right side of the gantry.
U and V axis are on the left side.
The lower front point is the origin.

### cnc control

The cnc shield used is the generic arduino uno 4 axis cnc shield https://www.neuhold-elektronik.at/catshop/product_info.php?products_id=7241

On the shield, the sockets X,Y,Z,A are used for X,Y,U,V axis

Jumpers set:
* D12/A.STEP
* D13/A.DIR
* All microstepping jumpers beneath the motor drivers

Motors and Power are connected on the shield.

Endstops:
The X and Y endstops are connected on the header.
U axis endstops are on the Z ones.
The V axis endstops are connected in parallel on the "CoolEn" pin.

A signal for turning the hotwire on/off is on the "Hold" pin.

## firmware

### settings

The signal placement can easily be changed within pindef.h

Defines for speed and steps/mm are in MotionProcessor.cpp

### building
Build and flash the firmware with platformIO IDE ( https://platformio.org/platformio-ide ) 
After flashing, all functionality is available via serial gcode commans


### Commands available:
  + G00/G01 move
  + G02/G04 pause
  + G28	home
  + G90	absolute coordinate mode
  + G91	relative coordinate mode
  + G92	set logical position

  + M3	hotwire on
  + M5	hotwire off
  + M17	enable motors
  + M18	disable motors
  + M100	help
  + M101	"dumb" line
  + M114	display current position

### firmware overview

The Firmware uses a ringbuffer to buffer incoming lines from the serial port.
Each line (=command) is interpreted one after another.
In the special case of the G00/G01 command, the next line is fetched right after the command has finished (via callback out of the last interrupt routine). Otherwise, the next command is started in the main loop.
The G00/G01 command utilizes the bresenham line drawing algorithm expanded to 4 axes in an interrupt to ensure accurate stepping of the motors. The dependency of the sum velocity of the moving wire on the angle of the line to move is compensated on the side where the faster axis is.

## operating the machine

To cut a piece, a line of gcode commands should be sent to the machine. These commands can be generated from various design programs/vector graphic converters.

Connect to the serial port with 115200 baud and send the gcodes, followed by '\n' (Enter).

For connecting and commanding the machine with a gcode file, check out https://github.com/stonetronics/stoneGcodeSender. A file containing a list of gcodes can be sent to a machine with this tool

For generating a file containing gcodes from svgs, check out https://github.com/stonetronics/hotwireGcodeGenerator. It can help generate the gcode files needed to cut desired shapes


