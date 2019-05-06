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
