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

<table border="1">
	<tr>
		<th> Command </th>
		<th> Implemented </th>
		<th> Tested </th>
		<th> Description </th>
	</tr>
	<td> <tr colspan="4"> Group 0: Configuration </tr> </td>
	<td> <tr> i2cCmd_GetAccelerateDecelerate  	</tr> <tr> Y </tr> <tr> </tr> <tr> 8 byte payload Slave -> Master + 1 Byte Status </tr> </td>
	<td> <tr> i2cCmd_SetAccelerateDecelerate  	</tr> <tr> Y </tr> <tr> </tr> <tr> 8 Byte payload Master -> Slave </tr> </td> </tr> </td>
	<td> <tr> i2cCmd_GetVMax  					</tr> <tr> Y </tr> <tr> </tr> <tr> 4 byte payload Slave -> Master + 1 Byte Status </tr> </td> </tr> </td>
	<td> <tr> i2cCmd_SetVMax  					</tr> <tr> Y </tr> <tr> </tr> <tr> 4 Byte payload Mater -> Slave </tr> </td>
	<td> <tr> i2cCmd_GetAlpha  					</tr> <tr> Y </tr> <tr> </tr> <tr> 4 Byte payload Slave -> Master + 1 Byte Status </tr> </td>
	<td> <tr> i2cCmd_SetAlpha  					</tr> <tr> Y </tr> <tr> </tr> <tr> 4 Byte payload Master -> Slave </tr> </td>
	<td> <tr> i2cCmd_GetMicrostepping  			</tr> <tr> </tr> <tr> </tr> <tr> 1 Byte payload Slave -> Master (2x 3 Bit) + 1 Byte Status </tr> </td>
	<td> <tr> i2cCmd_SetMicrostepping  			</tr> <tr> </tr> <tr> </tr> <tr> 1 Byte payload Master -> Slave </tr> </td>
	<td> <tr> i2cCmd_GetFault  					</tr> <tr> </tr> <tr> </tr> <tr> 1 Byte payload Master -> Slave (IS status) </tr> </td>
	<td> <tr> i2cCmd_RecalculateConstants  		</tr> <tr> Y </tr> <tr> </tr> <tr> Used to trigger recalculation of all constants (expensive operation; system should be stopped) </tr> </td>

	<td> <tr colspan="4"> Group 1: Queue </tr> </td>
	<td> <tr> i2cCmd_GetCommandQueueSize  		</tr> <tr> Y </tr> <tr> </tr> <tr> Get size (first byte) and unused entires (second byte) of command queue + 1 Byte status </tr> </td>

	<td> <tr colspan="4"> Group 2: Enqueue commands </tr> </td>
	<td> <tr> i2cCmd_Queue_Sync  				</tr> <tr> </tr> <tr> </tr> <tr> Sync. point; 1 Byte Channel </tr> </td>
	<td> <tr> i2cCmd_Queue_ConstSpeed  			</tr> <tr> </tr> <tr> </tr> <tr> Constant speed; 1 Byte Channel; 4 Byte Speed </tr> </td>
	<td> <tr> i2cCmd_Queue_MoveTo  				</tr> <tr> </tr> <tr> </tr> <tr> Move To (accelerated); 1 Byte Channel; 4 Byte Position </tr> </td>
	<td> <tr> i2cCmd_Queue_ConstSpeedAccel  	</tr> <tr> </tr> <tr> </tr> <tr> Constant speed with acceleration/deceleration; 1 Byte channel; 4 Byte speed </tr> </td>
	<td> <tr> i2cCmd_Queue_Hold  				</tr> <tr> </tr> <tr> </tr> <tr> Hold position; 1 byte channel </tr> </td>
	<td> <tr> i2cCmd_Queue_DisableDrv  			</tr> <tr> </tr> <tr> </tr> <tr> Disable drivers; 1 byte channel (both have to be ordered to disable to be effective) </tr> </td>

	<td> <tr colspan="4"> Group 3: Execute commands immedately </tr> </td>
	<td> <tr> i2cCmd_Exec_Sync  				</tr> <tr> </tr> <tr> </tr> <tr> Sync. point; 1 Byte Channel </tr> </td>
	<td> <tr> i2cCmd_Exec_ConstSpeed  			</tr> <tr> </tr> <tr> </tr> <tr> Constant speed; 1 Byte Channel; 4 Byte Speed </tr> </td>
	<td> <tr> i2cCmd_Exec_MoveTo  				</tr> <tr> </tr> <tr> </tr> <tr> Move To (accelerated); 1 Byte Channel; 4 Byte Position </tr> </td>
	<td> <tr> i2cCmd_Exec_ConstSpeedAccel  		</tr> <tr> </tr> <tr> </tr> <tr> Constant speed with acceleration/deceleration; 1 Byte channel; 4 Byte speed </tr> </td>
	<td> <tr> i2cCmd_Exec_Hold  				</tr> <tr> </tr> <tr> </tr> <tr> Hold position; 1 byte channel </tr> </td>
	<td> <tr> i2cCmd_Exec_DisableDrv  			</tr> <tr> </tr> <tr> </tr> <tr> Disable drivers; 1 byte channel (both have to be ordered to disable to be effective) </tr> </td>

	<td> <tr colspan="4"> Group F: Emergency commands </tr> </td>
	<td> <tr> i2cCmd_EmergencyStop  			</tr> <tr> </tr> <tr> </tr> <tr> Keeps motors engaged but stopped </tr> </td>
	<td> <tr> i2cCmd_EmergencyOff  				</tr> <tr> </tr> <tr> </tr> <tr> Keeps motors disabled </tr> </td>
</table>
