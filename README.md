# Dual stepper driver board (ATMEGA328P, DRV8825)

This repository contains sourcefiles and schematics for a DRV8825 based
dual stepper driver board. The stepper control algorithm is implemented
on an ATMega328P. The code is compatible with avr-gcc (see Makefile) and
the Arduino IDE (one has to ```#define ARDUINO 1``` to use code for the
Arduino platform).

This controller is thought to be used in robotic and CNC applications.
Some features make it useable as cruise control controller for small
robots that use steppers instead of other brushed or brushless motors
and which may require precise control over speed / sometimes precise control
over their position.

Since the controller uses I2C it can be controlled from 3.3V and 5V
devices without any level shifters (if pullups are only present to 3.3V
but not 5V vcc).

__The controller is currently work in progress__

The stepper driver can be controlled via I2C (__work in progress__) and
supports:

* Running at a constant speed
* Accelerated linear movement from one position to another
  position (stop to stop) as needed in many CNC applications
* Hold position (enable drivers, no movement)
* Free-wheeling (disable drivers)
* Accelerating or decelerating to another speed
* Emergency stop / Emergency stop with hold

Additional features:

* Commands and status via I2C to support up to 127 stepper
  driver boards (i.e. up to 254 steppers) per I2C bus.
* Runtime modification of acceleration/deceleration as well as
  maximum speed.
* Runtime reconfiguration of microstepping
* A configurable depth command queue (default 10 commands per channel)
* Aborting running commands
* Keeping track of current position (if enabled - useful for CNC applications)
* Keeping track of current speed

## Implementation status of I2C commands

|  Command | Implemented | Tested | Description |
| --- | --- | --- | --- |
| Group 0: Configuration |
| i2cCmd_GetAccelerateDecelerate  	|  Y |  |  8 byte payload Slave -> Master + 1 Byte Status |
| i2cCmd_SetAccelerateDecelerate  	|  Y |  |  8 Byte payload Master -> Slave |  |
| i2cCmd_GetVMax  					|  Y |  |  4 byte payload Slave -> Master + 1 Byte Status |  |
| i2cCmd_SetVMax  					|  Y |  |  4 Byte payload Mater -> Slave |
| i2cCmd_GetAlpha  					|  Y |  |  4 Byte payload Slave -> Master + 1 Byte Status |
| i2cCmd_SetAlpha  					|  Y |  |  4 Byte payload Master -> Slave |
| i2cCmd_GetMicrostepping  			|  |  |  1 Byte payload Slave -> Master (2x 3 Bit) + 1 Byte Status |
| i2cCmd_SetMicrostepping  			|  |  |  1 Byte payload Master -> Slave |
| i2cCmd_GetFault  					|  |  |  1 Byte payload Master -> Slave (IS status) |
| i2cCmd_RecalculateConstants  		|  Y |  |  Used to trigger recalculation of all constants (expensive operation; system should be stopped) |
| Group 1: Queue |
| i2cCmd_GetCommandQueueSize  		|  Y |  |  Get size (first byte) and unused entires (second byte) of command queue + 1 Byte status |
| Group 2: Enqueue commands |
| i2cCmd_Queue_Sync  				|  |  |  Sync. point; 1 Byte Channel |
| i2cCmd_Queue_ConstSpeed  			|  |  |  Constant speed; 1 Byte Channel; 4 Byte Speed |
| i2cCmd_Queue_MoveTo  				|  |  |  Move To (accelerated); 1 Byte Channel; 4 Byte Position |
| i2cCmd_Queue_ConstSpeedAccel  	|  |  |  Constant speed with acceleration/deceleration; 1 Byte channel; 4 Byte speed |
| i2cCmd_Queue_Hold  				|  |  |  Hold position; 1 byte channel |
| i2cCmd_Queue_DisableDrv  			|  |  |  Disable drivers; 1 byte channel (both have to be ordered to disable to be effective) |
| Group 3: Execute commands immedately |
| i2cCmd_Exec_Sync  				|  |  |  Sync. point; 1 Byte Channel |
| i2cCmd_Exec_ConstSpeed  			|  |  |  Constant speed; 1 Byte Channel; 4 Byte Speed |
| i2cCmd_Exec_MoveTo  				|  |  |  Move To (accelerated); 1 Byte Channel; 4 Byte Position |
| i2cCmd_Exec_ConstSpeedAccel  		|  |  |  Constant speed with acceleration/deceleration; 1 Byte channel; 4 Byte speed |
| i2cCmd_Exec_Hold  				|  |  |  Hold position; 1 byte channel |
| i2cCmd_Exec_DisableDrv  			|  |  |  Disable drivers; 1 byte channel (both have to be ordered to disable to be effective) |
| Group F: Emergency commands |
| i2cCmd_EmergencyStop  			|  |  |  Keeps motors engaged but stopped |
| i2cCmd_EmergencyOff  				|  |  |  Keeps motors disabled |
