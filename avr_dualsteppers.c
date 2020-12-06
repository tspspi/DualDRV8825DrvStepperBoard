#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/twi.h>
#include <stdint.h>

#define DEBUG

#ifndef STEPPER_I2C_ADDRESS
	#define STEPPER_I2C_ADDRESS 0x14
#endif
#ifndef M_PI
	#define M_PI 3.14159265358979323846264338327950288
#endif

/*
	If the following preprocessor directive is defined the steppers
	will be put in sleep mode upon reset (after initial power-up)
*/
#define STEPPER_DISABLE_STARTUP 1

#ifndef __cplusplus
	typedef int bool;
	#define true 1
	#define false 0
#endif

/*
	Pin usage:
		pinDir1		2	PD2		OUT
		pinStep1	3	PD3		OUT

		pinFault1	16	PC2		IN		PCINT10 (PCI1)
		pinDir2		14	PC0		OUT
		pinStep2	15	PC1		OUT
		pinFault2	17	PC3		IN		PCINT11 (PCI1)

		pinSleep    4	PD4		OUT
		pinReset    5	PD5		OUT
		pinMode2    6	PD6		OUT
		pinMode1    7	PD7		OUT

		pinMode0    8	PB0		OUT
		pinEnable   9	PB1		OUT


	I/O bank configuration:
		PB:		0	OUT		LOW (Fullstep)
				1	OUT		HIGH (ENABLE)
				2	IN		no pull
				3	IN		no pull
				4	IN		no pull
				5	IN		no pull
				6	IN		no pull
				7	IN		no pull

				DDRB = 0x03;
				PORTB = 0x02;

		PC:		0	OUT		LOW
				1	OUT		LOW
				2	IN		no pull
				3	IN		no pull
				4	IN		no pull
				5	IN		no pull
				6	IN		no pull
				7	IN		no pull

				DDRC = 0x03;
				PORTC = 0x00;

		PD:		0	IN		no pull
				1	IN		no pull
				2	OUT		LOW (Dir)
				3	OUT		LOW (Step)
				4	OUT		LOW (SLEEP)
				5	OUT		LOW (RESET)
				6	OUT		LOW (Fullstep)
				7	OUT		LOW (Fullstep)

				DDRD = 0xFC;
				PORTD = 0x00;
*/

/*
	Master clock source will be TIMER2

	Running with one of the following settings:
		Prescaler		Frequency		OCR1A		Effective Pulse Frequency
		/64				250 kHz			2			62.5 kHz
		/256			62.5 kHz		1			31.2 kHz
		/1024			15.625 kHz		1			7.812 kHz
*/

#ifndef STEPPER_COMMANDQUEUELENGTH
	#define STEPPER_COMMANDQUEUELENGTH 10
#endif

#define STEPPER_TIMERTICK_FRQ 			7812/2
#define STEPPER_TIMERTICK_PRESCALER		0x06
#define STEPPER_TIMERTICK_OVERFLOWVAL	0x01
#define STEPPER_WAKEUP_SLEEP_TICKS		10			/* More than 1.2 ms! */

/*
#define STEPPER_INITIAL_ALPHA			(0.01 * M_PI)
#define STEPPER_INITIAL_VMAX			(83.0 * M_PI)
#define STEPPER_INITIAL_ACCELERATION	(18.0 * M_PI)
#define STEPPER_INITIAL_DECELERATION	(-18.0 * M_PI)
*/
#define STEPPER_INITIAL_ALPHA			(0.01 * M_PI)
#define STEPPER_INITIAL_VMAX			6
#define STEPPER_INITIAL_ACCELERATION	1 /* 6 */
#define STEPPER_INITIAL_DECELERATION	-1 /* -6 */
#define STEPPER_INITIAL_MICROSTEPPING	0

/*
	Min and max configureable values
		Alpha:		Step size in radians
		V_max:		Maximum speed in radians per second
					Note: Since max. 250 kHz stepping frequency
						is supported by DRV8825 we use this as
						the limit together with alpha.

						(Alpha * 250 000)
		a, d		Acceleration and deceleration in radians per
					squaresecond
*/
#define STEPPER_MAX_ALPHA					(0.01 * M_PI)
#define STEPPER_MIN_ALPHA					((0.01 * M_PI)/128.0)
#define STEPPER_MAX_VMAX					100 /* (STEPPER_MAX_ALPHA * 250000.0) */
#define STEPPER_MIN_VMAX					0.01
// #define STEPPER_MIN_ACCELERATION			0.01 /* Currently arbitrary! */
// #define STEPPER_MAX_ACCELERATION			((STEPPER_MAX_VMAX * STEPPER_MAX_VMAX)/(STEPPER_MIN_ALPHA*4294967296.0))
#define STEPPER_MIN_ACCELERATION			((STEPPER_MAX_VMAX * STEPPER_MAX_VMAX)/(STEPPER_MIN_ALPHA*4294967296.0))
#define STEPPER_MAX_ACCELERATION			1000 /* Currently arbitrary */
#define STEPPER_MIN_DECELERATION			(-1.0 * ((STEPPER_MAX_VMAX * STEPPER_MAX_VMAX)/(STEPPER_MIN_ALPHA*4294967296.0)))
#define STEPPER_MAX_DECELERATION			-0.01 /* Currently arbitrary! */

#define STEPPER_MIN_RELATIVEDISTANCE		STEPPER_MIN_ALPHA
#define STEPPER_MAX_RELATIVEDISTANCE		(4294967295.0 * STEPPER_MIN_ALPHA)

/*
	We use our own systick implementation
*/
unsigned long int millis();
unsigned long int micros();
void delay(unsigned long millisecs);
void delayMicros(unsigned int microDelay);

enum stepperCommandType {
	stepperCommand_AccelerateStopToStop,
	stepperCommand_ConstantSpeed,
	stepperCommand_Stop,
	stepperCommand_Disable,
	stepperCommand_AccelerateToSpeed,
	stepperCommand_DecelerateToSpeed,
};

struct stepperCommand {
	enum stepperCommandType		cmdType;
	int 						forward;
	union {
		struct {
			uint32_t			nA;
			uint32_t			nC;
			uint32_t			nD;

			double				c7, c8, c9; /* Calculation constants for our specific acceleration and deceleration. */

			double				initialDelayTicks;
		} acceleratedStopToStop;
		struct {
			uint32_t			cConst; /* In case of constant speed mode we supply the tick count per constant step */
		} constantSpeed;
		struct {
			uint32_t			nA;
			uint32_t			nD;

			double				c8, c9;
			double				cEnd;
			double				cStart;
		} accelerateDecelerateToConstSpeed;
	} data;
};

/*
	Command queue is a ringbuffer. Head points always to the next entry
	that is available for writing. If head == tail the queue is EMPTY!

	Tail is only ever advanced by TIMER2_COMPA_vect, head only ever by
	the motion planner.

	If Head + 1 == Tail the queue is currently FULL and nothing can
	be enqueued.
*/
struct stepperState {
	double							c_i;				/* The previously loaded c_i (used during calculation and advancing) */
	uint32_t						counterCurrent;		/* Counter reduced every time until we reach zero */

	struct stepperCommand			cmdQueue[STEPPER_COMMANDQUEUELENGTH];
	unsigned int					cmdQueueHead;
	unsigned int					cmdQueueTail;

	#ifdef ENABLE_ABSOLUTEPOSITION
		long int						currentPosition;			/* Current position in "steps" i.e. currentPosition * alpha = angular Position */
	#endif

	/* Cache for precalculated constants - see documentation */
	struct {
		double						c1;
		double						c2;
		double						c3;
		double						c4;
		double						c5;

		double						c7;
		double						c8;
		double						c9;
		double						c10;
	} constants;

	struct {
		double						acceleration; /* in rad/sec */
		double						deceleration; /* in rad/sec */
		double						alpha; /* step size in rad */
		double						vmax; /* in rad/sec */
	} settings;
};

#define STEPPER_COUNT 2

/*
	Declare state for both steppers
*/
static struct stepperState			state[STEPPER_COUNT];
static uint8_t						stateMicrostepping;
volatile static uint8_t				stateFault;
static uint8_t						drvEnableState;
static uint8_t						drvRealEnabled;

bool bResetRun = false;
static volatile bool bIntTriggered = false;

/*@
	requires \valid(&PORTC) && \valid(&PORTD);

	assigns drvRealEnabled;
	assigns bIntTriggered;

	assigns PORTC, PORTD;

	ensures
		\forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].cmdQueueHead >= 0) && (state[iStep].cmdQueueHead < STEPPER_COMMANDQUEUELENGTH);
	ensures
		\forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].cmdQueueTail >= 0) && (state[iStep].cmdQueueTail < STEPPER_COMMANDQUEUELENGTH);
*/
static void handleTimer2Interrupt() {
	if(!bIntTriggered) {
		return;
	}
	bIntTriggered = false;

	/*
		Re-enable counter. This is used to delay the
		next stepper commands after re-awaking from
		sleep.
	*/
	if((drvRealEnabled & 0x7F) != 0) {
		drvRealEnabled = drvRealEnabled - 1;
		return;
	}

	/*
		Interrupt handler executed with 2 * STEPPER_TIMERTICK_FRQ frequency
		either use to disable step pins or advance state machine depending on
		out state. We have to finish in way less then 1024 ticks ...
	*/
	int stepperIdx;
	/*@
		loop assigns PORTD, PORTC;
		loop assigns drvRealEnabled, drvEnableState;
		loop assigns state[0 .. (STEPPER_COUNT-1)];

		loop invariant 0 <= stepperIdx < STEPPER_COUNT;

	*/
	for(stepperIdx = 0; stepperIdx < STEPPER_COUNT; stepperIdx = stepperIdx + 1) {
		if(bResetRun) {
			if(stepperIdx == 0) {
				// Pulse PD2 low
				PORTD = PORTD & (~0x08);
			} else {
				// Pulse PC1 low
				PORTC = PORTC & (~0x02);
			}

			if((drvEnableState == 0) && ((drvRealEnabled & 0x80) != 0)) {
				/* Put driver into sleep if none is enabled AND our current real enable state is not the same */
				drvRealEnabled = drvRealEnabled & 0x7F;
				PORTD = PORTD & (~0x10); /* Enter sleep state by pulling SLEEP LOW (enabled) */
			}
		} else {
			if(state[stepperIdx].cmdQueueTail == state[stepperIdx].cmdQueueHead) {
				/* Nothing to do ... */
				continue;
			}

			const int qIdx = state[stepperIdx].cmdQueueTail;
			if(state[stepperIdx].c_i == -1) {
				/* We wake up from idle ... any may have to do some initialization */
				if(stepperIdx == 0) {
					// PD2
					if(state[stepperIdx].cmdQueue[qIdx].forward != 0) {
						PORTD = PORTD | 0x04;
					} else {
						PORTD = PORTD & (~0x04);
					}
				} else {
					// PC0
					if(state[stepperIdx].cmdQueue[qIdx].forward != 0) {
						PORTC = PORTC | 0x01;
					} else {
						PORTC = PORTC & (~0x01);
					}
				}

				/* Check if drivers are disabled and we have to execute an command ... */
				if((drvRealEnabled & 0x80) == 0) {
					/* Steppers are disabled. Re-enable and wait for startup period */
					PORTD = PORTD | 0x10; /* Leave sleep state by pulling SLEEP HIGH (disabled) */
					drvRealEnabled = 0x80 | STEPPER_WAKEUP_SLEEP_TICKS;
					drvEnableState = drvEnableState | (1 << stepperIdx);
					break; // Next stepper as usual
				}

				if(state[stepperIdx].cmdQueue[qIdx].cmdType == stepperCommand_AccelerateStopToStop) {
					state[stepperIdx].c_i = state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.initialDelayTicks;
					if(state[stepperIdx].c_i > 4294967295.0) {
						state[stepperIdx].counterCurrent = ~0;
					} else if(state[stepperIdx].c_i < 1.0) {
						state[stepperIdx].counterCurrent = 1;
					} else {
						state[stepperIdx].counterCurrent = (uint32_t)state[stepperIdx].c_i;
					}
				} else if(state[stepperIdx].cmdQueue[qIdx].cmdType == stepperCommand_ConstantSpeed) {
					state[stepperIdx].c_i = state[stepperIdx].cmdQueue[qIdx].data.constantSpeed.cConst;
					state[stepperIdx].counterCurrent = state[stepperIdx].cmdQueue[qIdx].data.constantSpeed.cConst;
				} else if(state[stepperIdx].cmdQueue[qIdx].cmdType == stepperCommand_Disable) {
					drvEnableState = drvEnableState & (~(0x01 << stepperIdx));
					/* Switch to next state at next interrupt */
					state[stepperIdx].cmdQueueTail = (state[stepperIdx].cmdQueueTail + 1) % STEPPER_COMMANDQUEUELENGTH;
					state[stepperIdx].c_i = -1; /* This will initialize the next state as soon as it is available */
					continue;
				} else if(state[stepperIdx].cmdQueue[qIdx].cmdType == stepperCommand_Stop) {
					/*
						Our driver has already been enabled since we do that whenever we
						reach an enqueued command. So simply remove this task from the task
						queue. As long as nothing has been enqueued the steppers will be
						engaged and stopped
					*/
					state[stepperIdx].cmdQueueTail = (state[stepperIdx].cmdQueueTail + 1) % STEPPER_COMMANDQUEUELENGTH;
					state[stepperIdx].c_i = -1; /* This will initialize the next state as soon as it is available */
					continue;
				} else if((state[stepperIdx].cmdQueue[qIdx].cmdType == stepperCommand_AccelerateToSpeed) || (state[stepperIdx].cmdQueue[qIdx].cmdType == stepperCommand_DecelerateToSpeed)) {
					state[stepperIdx].c_i = state[stepperIdx].cmdQueue[qIdx].data.accelerateDecelerateToConstSpeed.cStart;
					if(state[stepperIdx].c_i > 4294967295.0) {
						state[stepperIdx].counterCurrent = ~0;
					} else if(state[stepperIdx].c_i < 1.0) {
						state[stepperIdx].counterCurrent = 1;
					} else {
						state[stepperIdx].counterCurrent = (uint32_t)state[stepperIdx].c_i;
					}
				} else {
					continue;
				}
			} else if((state[stepperIdx].counterCurrent = state[stepperIdx].counterCurrent - 1) > 0) {
				/* Just advance our counter ... */
				continue;
			} else if(state[stepperIdx].cmdQueue[qIdx].cmdType == stepperCommand_AccelerateStopToStop) {
				if(state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nA > 0) {
					state[stepperIdx].c_i = state[stepperIdx].c_i / (1 + state[stepperIdx].constants.c8 * state[stepperIdx].c_i * state[stepperIdx].c_i);
					state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nA = state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nA - 1;

					/* c_i to counter value */
					if(state[stepperIdx].c_i > 4294967295.0) {
						state[stepperIdx].counterCurrent = ~0;
					} else if(state[stepperIdx].c_i < 1.0) {
						state[stepperIdx].counterCurrent = 1;
					} else {
						state[stepperIdx].counterCurrent = (uint32_t)state[stepperIdx].c_i;
					}
				} else if(state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nC > 0) {
					state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nC = state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nC - 1;

					/* c_i to counter value */
					if(state[stepperIdx].c_i > 4294967295.0) {
						state[stepperIdx].counterCurrent = ~0;
					} else if(state[stepperIdx].c_i < 1.0) {
						state[stepperIdx].counterCurrent = 1;
					} else {
						state[stepperIdx].counterCurrent = (uint32_t)state[stepperIdx].c_i;
					}
				} else if(state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nD > 0) {
					state[stepperIdx].c_i = state[stepperIdx].c_i / (1 + state[stepperIdx].constants.c9 * state[stepperIdx].c_i * state[stepperIdx].c_i);
					/* c_i to counter value */
					if(state[stepperIdx].c_i > 4294967295.0) {
						state[stepperIdx].counterCurrent = ~0;
					} else if(state[stepperIdx].c_i < 1.0) {
						state[stepperIdx].counterCurrent = 1;
					} else {
						state[stepperIdx].counterCurrent = (uint32_t)state[stepperIdx].c_i;
					}

					if((state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nD = state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nD - 1) == 0) {
						/* Switch to next state at next interrupt */
						state[stepperIdx].cmdQueueTail = (state[stepperIdx].cmdQueueTail + 1) % STEPPER_COMMANDQUEUELENGTH;
						state[stepperIdx].c_i = -1; /* This will initialize the next state as soon as it is available */
					}
				}
			} else if(state[stepperIdx].cmdQueue[qIdx].cmdType == stepperCommand_AccelerateToSpeed) {
				state[stepperIdx].c_i = state[stepperIdx].c_i / (1 + state[stepperIdx].constants.c8 * state[stepperIdx].c_i * state[stepperIdx].c_i);

				/* c_i to counter value */
				if(state[stepperIdx].c_i > 4294967295.0) {
					state[stepperIdx].counterCurrent = ~0;
				} else if(state[stepperIdx].c_i < 1.0) {
					state[stepperIdx].counterCurrent = 1;
				} else {
					state[stepperIdx].counterCurrent = (uint32_t)state[stepperIdx].c_i;
				}

				if((state[stepperIdx].cmdQueue[qIdx].data.accelerateDecelerateToConstSpeed.nA = state[stepperIdx].cmdQueue[qIdx].data.accelerateDecelerateToConstSpeed.nA - 1) == 0) {
					/* Switch to next state at next interrupt */
					state[stepperIdx].cmdQueueTail = (state[stepperIdx].cmdQueueTail + 1) % STEPPER_COMMANDQUEUELENGTH;
					state[stepperIdx].c_i = -1; /* This will initialize the next state as soon as it is available */
				}
			} else if(state[stepperIdx].cmdQueue[qIdx].cmdType == stepperCommand_DecelerateToSpeed) {
				state[stepperIdx].c_i = state[stepperIdx].c_i / (1 + state[stepperIdx].constants.c9 * state[stepperIdx].c_i * state[stepperIdx].c_i);

				/* c_i to counter value */
				if(state[stepperIdx].c_i > 4294967295.0) {
					state[stepperIdx].counterCurrent = ~0;
				} else if(state[stepperIdx].c_i < 1.0) {
					state[stepperIdx].counterCurrent = 1;
				} else {
					state[stepperIdx].counterCurrent = (uint32_t)state[stepperIdx].c_i;
				}

				if((state[stepperIdx].cmdQueue[qIdx].data.accelerateDecelerateToConstSpeed.nD = state[stepperIdx].cmdQueue[qIdx].data.accelerateDecelerateToConstSpeed.nD - 1) == 0) {
					/* Switch to next state at next interrupt */
					state[stepperIdx].cmdQueueTail = (state[stepperIdx].cmdQueueTail + 1) % STEPPER_COMMANDQUEUELENGTH;
					state[stepperIdx].c_i = -1; /* This will initialize the next state as soon as it is available */
				}
			} else if(state[stepperIdx].cmdQueue[qIdx].cmdType == stepperCommand_ConstantSpeed) {
				state[stepperIdx].counterCurrent = state[stepperIdx].c_i; // Counter reload ...
			}

			if(stepperIdx == 0) {
				// Pulse PD2 high
				PORTD = PORTD | 0x08;
				#ifdef ENABLE_ABSOLUTEPOSITION
					if((PORTD & 0x04) != 0) {
						if(state[0].currentPosition < 2147483647) {
							state[0].currentPosition++;
						}
					} else {
						if(state[0].currentPosition > 0) {
							state[0].currentPosition--;
						}
					}
				#endif
			} else {
				// Pulse PC1 high
				PORTC = PORTC | 0x02;
				#ifdef ENABLE_ABSOLUTEPOSITION
					if((PORTC & 0x01) != 0) {
						if(state[1].currentPosition < 2147483647) {
							state[1].currentPosition++;
						}
					} else {
						if(state[1].currentPosition > 0) {
							state[1].currentPosition--;
						}
					}
				#endif
			}
		}
	}
	bResetRun = !bResetRun;
}

/*@
	assigns bIntTriggered;
	ensures bIntTriggered == true;
*/
ISR(TIMER2_COMPA_vect) {
	bIntTriggered = true;
}

/*@
	requires \valid_read(&PINC);

	assigns stateFault;
*/
ISR(PCINT1_vect) {
	/*
		Update fault pins ...
	*/
	uint8_t faultPins = PINC;
	stateFault = stateFault | (((faultPins >> 2) & 0x01) ^ 0x01) | (((faultPins >> 2) & 0x02) ^ 0x02);
}

/*
	c2		Trapezoidal profile		n_acceleration = v_d^2 / (2 * a * \alpha)
	c4		Trapezoidal profile		n_deceleration = v_d^2 / (2 * d * \alpha)
	c5		Triangular profile		d / (d + a)

	c7		Time stepping: Initial time step delay squared (dt_0 * dt_0)
	c8		Time stepping constant R_a for acceleration
	c9		Time stepping constant R_d for deceleration
	c10		Constant speed timestep constant (alpha * f)
*/
/*@
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
	 	==> (state[iStep].settings.vmax >= STEPPER_MIN_VMAX) && (state[iStep].settings.vmax <= STEPPER_MAX_VMAX);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].settings.alpha >= STEPPER_MIN_ALPHA) && (state[iStep].settings.alpha <= STEPPER_MAX_ALPHA);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].settings.acceleration >= STEPPER_MIN_ACCELERATION) && (state[iStep].settings.acceleration <= STEPPER_MAX_ACCELERATION);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==>  (state[iStep].settings.deceleration >= STEPPER_MIN_DECELERATION) && (state[iStep].settings.deceleration <= STEPPER_MAX_DECELERATION);

	behavior unknownChannel:
		assumes (stepperIndex < 0) || (stepperIndex >= STEPPER_COUNT);
		assigns \nothing;
	behavior knownChannel:
		assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);

		assigns state[stepperIndex].constants.c1,
		 	state[stepperIndex].constants.c2,
			state[stepperIndex].constants.c3,
			state[stepperIndex].constants.c4,
			state[stepperIndex].constants.c5,
			state[stepperIndex].constants.c7,
			state[stepperIndex].constants.c8,
			state[stepperIndex].constants.c9,
			state[stepperIndex].constants.c10;

		ensures state[stepperIndex].constants.c2 == (state[stepperIndex].settings.vmax * state[stepperIndex].settings.vmax) / (2 * state[stepperIndex].settings.acceleration * state[stepperIndex].settings.alpha);
		ensures state[stepperIndex].constants.c4 == -1 * (state[stepperIndex].settings.vmax * state[stepperIndex].settings.vmax) / (2 * state[stepperIndex].settings.deceleration * state[stepperIndex].settings.alpha);
		ensures state[stepperIndex].constants.c5 == state[stepperIndex].settings.deceleration / (state[stepperIndex].settings.deceleration - state[stepperIndex].settings.acceleration);
		ensures state[stepperIndex].constants.c7 == 2 * state[stepperIndex].settings.alpha / state[stepperIndex].settings.acceleration;
		ensures state[stepperIndex].constants.c8 == state[stepperIndex].settings.acceleration / (state[stepperIndex].settings.alpha * (double)STEPPER_TIMERTICK_FRQ * (double)STEPPER_TIMERTICK_FRQ);
		ensures state[stepperIndex].constants.c9 == state[stepperIndex].settings.acceleration / (state[stepperIndex].settings.alpha * (double)STEPPER_TIMERTICK_FRQ * (double)STEPPER_TIMERTICK_FRQ);
		ensures state[stepperIndex].constants.c10 == state[stepperIndex].settings.alpha * (double)STEPPER_TIMERTICK_FRQ;

	disjoint behaviors;
	complete behaviors;
*/
static bool updateConstants(int stepperIndex) {
	/*
		Expensive update of constants for motion planner
	*/
	if((stepperIndex >= STEPPER_COUNT) || (stepperIndex < 0)) {
		return false;
	}

	state[stepperIndex].constants.c1 = 1.0 / (2.0 * state[stepperIndex].settings.acceleration * state[stepperIndex].settings.alpha);
	state[stepperIndex].constants.c2 = state[stepperIndex].settings.vmax * state[stepperIndex].settings.vmax * state[stepperIndex].constants.c1; /*@ assert (state[stepperIndex].constants.c2 < 4294967296) && (state[stepperIndex].constants.c2 >= 0); */
	state[stepperIndex].constants.c3 = 1.0 / (2.0 * state[stepperIndex].settings.deceleration * state[stepperIndex].settings.alpha);
	state[stepperIndex].constants.c4 = -1.0 * state[stepperIndex].settings.vmax * state[stepperIndex].settings.vmax * state[stepperIndex].constants.c3; /*@ assert (state[stepperIndex].constants.c4 < 4294967296) && (state[stepperIndex].constants.c4 >= 0); */
	state[stepperIndex].constants.c5 = state[stepperIndex].settings.deceleration / (state[stepperIndex].settings.deceleration - state[stepperIndex].settings.acceleration);
	state[stepperIndex].constants.c7 = 2 * state[stepperIndex].settings.alpha / state[stepperIndex].settings.acceleration;
	state[stepperIndex].constants.c8 = state[stepperIndex].settings.acceleration / (state[stepperIndex].settings.alpha * (double)STEPPER_TIMERTICK_FRQ * (double)STEPPER_TIMERTICK_FRQ);
	state[stepperIndex].constants.c9 = state[stepperIndex].settings.deceleration / (state[stepperIndex].settings.alpha * (double)STEPPER_TIMERTICK_FRQ * (double)STEPPER_TIMERTICK_FRQ);
	state[stepperIndex].constants.c10 = state[stepperIndex].settings.alpha * (double)STEPPER_TIMERTICK_FRQ;

	return true;
}

/*@
	requires \valid(&PORTB) && \valid(&PORTD);

	behavior supportedMicrosteps:
		assumes (microsteps == 0) || (microsteps == 2) || (microsteps == 4)
			|| (microsteps == 8) || (microsteps == 16) || (microsteps == 32);

		assigns PORTB;
		assigns PORTD;
		assigns stateMicrostepping;

		ensures stateMicrostepping == microsteps;
	behavior unsupportedMicrosteps:
		assumes (microsteps != 0) && (microsteps != 2) && (microsteps != 4)
			&& (microsteps != 8) && (microsteps != 16) && (microsteps != 32);

		assigns \nothing;
	disjoint behaviors;
	complete behaviors;
*/
static void stepperSetMicrostepping(uint8_t microsteps) {
	/*
		pinMode0    8	PB0		OUT
		pinMode2    6	PD6		OUT
		pinMode1    7	PD7		OUT

		M0	M1	M2		Steps
		0	0	0		Full steps
		1	0	0		1/2
		0	1	0		1/4
		1	1	0		1/8
		0	0	1		1/16
		1	0	1		1/32
		0	1	1		Undefined (1/64)
		1	1	1		Undefined (1/128)
	*/

	switch(microsteps) {
		case 0:			stateMicrostepping =  0; PORTB = PORTB & (~(0x01)); PORTD = PORTD & 0x3F; 						 break;	/* PB0: 0, PD7: 0, PD6: 0 */
		case 2:			stateMicrostepping =  2; PORTB = PORTB |     0x01 ; PORTD = PORTD & 0x3F;						 break;	/* PB0: 1, PD7: 0, PD6: 0 */
		case 4:			stateMicrostepping =  4; PORTB = PORTB & (~(0x01)); PORTD = PORTD & 0x3F; PORTD = PORTD | 0x80; break;	/* PB0: 0, PD7: 1, PD6: 0 */
		case 8:			stateMicrostepping =  8; PORTB = PORTB |     0x01 ; PORTD = PORTD & 0x3F; PORTD = PORTD | 0x80; break;	/* PB0: 1, PD7: 1, PD6: 0 */
		case 16:		stateMicrostepping = 16; PORTB = PORTB & (~(0x01)); PORTD = PORTD & 0x3F; PORTD = PORTD | 0x40; break;	/* PB0: 0, PD7: 0, PD6: 1 */
		case 32:		stateMicrostepping = 32; PORTB = PORTB |     0x01 ; PORTD = PORTD & 0x3F; PORTD = PORTD | 0x40; break;	/* PB0: 1, PD7: 0, PD6: 1 */
		default:		break;
	}
}

/*@
	requires \valid(&TCCR2B) && \valid(&TCNT2) && \valid(&TCCR2A) && \valid(&OCR2A) && \valid(&TIMSK2);
	requires \valid(&PORTB) && \valid(&DDRB);
	requires \valid(&PORTC) && \valid(&DDRC) && \valid(&PINC);
	requires \valid(&PORTD) && \valid(&DDRD);
	requires \valid(&PCICR) && \valid(&PCMSK1);

	assigns stateFault;
	assigns drvEnableState;
	assigns drvRealEnabled;

	ensures
		\forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].cmdQueueHead == 0) && (state[iStep].cmdQueueTail == 0) &&
		    (state[iStep].counterCurrent == 0) && (state[iStep].c_i == -1) &&
		    (state[iStep].settings.acceleration == STEPPER_INITIAL_ACCELERATION) &&
		    (state[iStep].settings.deceleration == STEPPER_INITIAL_DECELERATION) &&
		    (state[iStep].settings.alpha == STEPPER_INITIAL_ALPHA) &&
		    (state[iStep].settings.vmax == STEPPER_INITIAL_VMAX);

	ensures
	 	\forall integer stepperIndex; 0 <= stepperIndex < STEPPER_COUNT
		==> (state[stepperIndex].settings.vmax >= STEPPER_MIN_VMAX) && (state[stepperIndex].settings.vmax <= STEPPER_MAX_VMAX)
			&& (state[stepperIndex].settings.alpha >= STEPPER_MIN_ALPHA) && (state[stepperIndex].settings.alpha <= STEPPER_MAX_ALPHA)
			&& (state[stepperIndex].settings.acceleration >= STEPPER_MIN_ACCELERATION) && (state[stepperIndex].settings.acceleration <= STEPPER_MAX_ACCELERATION)
			&& (state[stepperIndex].settings.deceleration >= STEPPER_MIN_DECELERATION) && (state[stepperIndex].settings.deceleration <= STEPPER_MAX_DECELERATION);
*/
static void stepperSetup() {
	/*
		Stop our timer if it's currently running
	*/
	#ifndef FRAMAC_SKIP
		cli();
	#endif
	TCCR2B = 0; 							/* Disable timer if it's currently running */
	#ifndef FRAMAC_SKIP
		sei();
	#endif

	/*
		Initialize pins with usage as described above
		Pull enable HIGH (inputs disabled), RESET and SLEEP low (ENABLED)
		Microstepping initally set to full step
	*/
	DDRB = 0x03;
	PORTB = 0x02;
	DDRC = 0x03;
	PORTC = 0x00;
	DDRD = 0xFC;
	PORTD = 0x00;

	stateMicrostepping = 0;

	/* Enable pin change interrupts for FAULT pins */
	PCICR = 0x02;		/* Set PCIE1 (PCIE0 and PCIE2 are disabled) */
	PCMSK1 = 0x0C;		/* Mask only for PCINT10, PCINT11 i.e. PC2, PC3) */

	/* Perform reset and initialization */
	delay(10);
	PORTD = PORTD | 0x10; /* Leave sleep state by pulling SLEEP HIGH (disabled) */
	delay(2);
	PORTB = PORTB & (~0x02); /* Enable inputs by pulling ENABLE LOW (enabled) */
	delay(150);
	PORTD = PORTD | 0x20; /* Leave reset mode by pulling RESET HIGH (disabled) */
	delay(150);

	/*
		Initialize stepper state machine to zero
	*/
	int i;
	/*@
		loop assigns state[0 .. (STEPPER_COUNT-1)];
		loop invariant 0 <= i < STEPPER_COUNT;
	*/
	for(i = 0; i < STEPPER_COUNT; i=i+1) {
		state[i].cmdQueueHead = 0;
		state[i].cmdQueueTail = 0;
		state[i].counterCurrent = 0;
		state[i].c_i = -1; /* Will trigger initialization after planning */

		#ifdef ENABLE_ABSOLUTEPOSITION
			state[i].currentPosition = 0; /* Initialize always at position 0 */
		#endif

		state[i].settings.acceleration = STEPPER_INITIAL_ACCELERATION;
		state[i].settings.deceleration = STEPPER_INITIAL_DECELERATION;
		state[i].settings.alpha = STEPPER_INITIAL_ALPHA;
		state[i].settings.vmax = STEPPER_INITIAL_VMAX;

		updateConstants(i);
	}

	drvEnableState = 0x03; /* Both steppers enabled after initialization. */
	drvRealEnabled = 0x80; /* Both steppers are enabled and we do not need any grace period till we can perform some work ... */

	/* Fault status to status register */
	{
		#ifndef FRAMAC_SKIP
			cli();
		#endif
		uint8_t faultPins = PINC;
		stateFault = (((faultPins >> 2) & 0x01) ^ 0x01) | (((faultPins >> 2) & 0x02) ^ 0x02);
		#ifndef FRAMAC_SKIP
			sei();
		#endif
	}

	/*
		Initialize our timer. We use Timer2 for our purposes
	*/
	TCNT2 = 0; 								/* Set current timer counter to zero */
	TCCR2A = 0x02;							/* CTC Mode (count up to OCR2A), disable OCR output pins */
	OCR2A = 0x01;							/* We count up to one - so trigger every pulse */
	TIMSK2 = 0x02;							/* Set OCIE2A flag to enable interrupts on output compare */
	TCCR2B = STEPPER_TIMERTICK_PRESCALER;	/* Select our prescaler, non FOCA, enable timer */

	#ifdef STEPPER_DISABLE_STARTUP
		drvEnableState = 0x00; /* Steppers will be disabled on startup */
	#endif
}

/*@
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].settings.vmax >= STEPPER_MIN_VMAX) && (state[iStep].settings.vmax <= STEPPER_MAX_VMAX);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].cmdQueueHead >= 0) && (state[iStep].cmdQueueHead < STEPPER_COMMANDQUEUELENGTH);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].constants.c10 == state[iStep].settings.alpha * STEPPER_TIMERTICK_FRQ);
	requires (immediate == true) || (immediate == false);

	behavior unknownStepper:
		assumes (stepperIndex < 0) || (stepperIndex >= STEPPER_COUNT);
		assigns \nothing;
	behavior knownStepperImmediate:
		assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
		assumes immediate == true;

		requires (v >= (state[stepperIndex].settings.alpha * STEPPER_TIMERTICK_FRQ)/(4294967295)) && (v <= state[stepperIndex].settings.alpha * STEPPER_TIMERTICK_FRQ);

		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.constantSpeed.cConst;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].forward;
		assigns state[stepperIndex].cmdQueueHead;
		assigns state[stepperIndex].cmdQueueTail;

		ensures state[stepperIndex].cmdQueueTail == \old(state[stepperIndex].cmdQueueHead);
		ensures state[stepperIndex].cmdQueueHead == ((\old(state[stepperIndex].cmdQueueHead) + 1) % STEPPER_COMMANDQUEUELENGTH);
		ensures state[stepperIndex].c_i == -1;
	behavior knownStepperPlanned:
		assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
		assumes immediate == false;

		requires (v >= (state[stepperIndex].settings.alpha * STEPPER_TIMERTICK_FRQ)/(4294967295)) && (v <= state[stepperIndex].settings.alpha * STEPPER_TIMERTICK_FRQ);

		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.constantSpeed.cConst;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].forward;
		assigns state[stepperIndex].cmdQueueHead;

		ensures state[stepperIndex].cmdQueueHead == ((\old(state[stepperIndex].cmdQueueHead) + 1) % STEPPER_COMMANDQUEUELENGTH);
	disjoint behaviors;
	complete behaviors;
*/
static void stepperPlanMovement_ConstantSpeed(int stepperIndex, double v, int direction, bool immediate) {
	if((stepperIndex < 0) || (stepperIndex >= STEPPER_COUNT)) { return; }

	/*
		Determine at which index we want to plan
	*/
	const int idx = state[stepperIndex].cmdQueueHead;

	state[stepperIndex].cmdQueue[idx].cmdType = stepperCommand_ConstantSpeed;

	double vMin = state[stepperIndex].settings.alpha * STEPPER_TIMERTICK_FRQ / (4294967295);
	if(v < vMin) { v = vMin; }
	else if(v > state[stepperIndex].settings.vmax) { v = state[stepperIndex].settings.vmax; }

	double tickCount = state[stepperIndex].constants.c10 / v;
	if(tickCount > 4294967295.0) {
		/* Clamp to maximum */
		state[stepperIndex].cmdQueue[idx].data.constantSpeed.cConst = ~0;
	} else if(tickCount < 1.0) {
		/* Clamp to minimum */
		state[stepperIndex].cmdQueue[idx].data.constantSpeed.cConst = 1;
	} else {
		state[stepperIndex].cmdQueue[idx].data.constantSpeed.cConst = (uint32_t)(tickCount);
	}

	state[stepperIndex].cmdQueue[idx].forward = direction;

	/*
		Make command active
	*/
	if(immediate) {
		state[stepperIndex].cmdQueueTail = state[stepperIndex].cmdQueueHead;
		state[stepperIndex].c_i = -1;
		state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
	} else {
		state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
	}

	return;
}

/*@
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
                ==> (state[iStep].cmdQueueHead >= 0) && (state[iStep].cmdQueueHead < STEPPER_COMMANDQUEUELENGTH);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> state[iStep].constants.c2 == (state[iStep].settings.vmax * state[iStep].settings.vmax) / (2 * state[iStep].settings.acceleration * state[iStep].settings.alpha);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> state[iStep].constants.c4 == -1 * (state[iStep].settings.vmax * state[iStep].settings.vmax) / (2 * state[iStep].settings.deceleration * state[iStep].settings.alpha);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> state[iStep].constants.c5 == state[iStep].settings.deceleration / (state[iStep].settings.deceleration - state[iStep].settings.acceleration);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> state[iStep].constants.c7 == 2 * state[iStep].settings.alpha / state[iStep].settings.acceleration;
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> state[iStep].constants.c8 == state[iStep].settings.acceleration / (state[iStep].settings.alpha * (double)STEPPER_TIMERTICK_FRQ * (double)STEPPER_TIMERTICK_FRQ);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> state[iStep].constants.c9 == state[iStep].settings.acceleration / (state[iStep].settings.alpha * (double)STEPPER_TIMERTICK_FRQ * (double)STEPPER_TIMERTICK_FRQ);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> state[iStep].constants.c10 == state[iStep].settings.alpha * (double)STEPPER_TIMERTICK_FRQ;

	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> (state[iStep].settings.vmax >= STEPPER_MIN_VMAX) && (state[iStep].settings.vmax <= STEPPER_MAX_VMAX);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> (state[iStep].settings.alpha >= STEPPER_MIN_ALPHA) && (state[iStep].settings.alpha <= STEPPER_MAX_ALPHA);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> (state[iStep].settings.acceleration >= STEPPER_MIN_ACCELERATION) && (state[iStep].settings.acceleration <= STEPPER_MAX_ACCELERATION);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> (state[iStep].settings.deceleration >= STEPPER_MIN_DECELERATION) && (state[iStep].settings.deceleration <= STEPPER_MAX_DECELERATION);

	requires (sTotal >= STEPPER_MIN_RELATIVEDISTANCE);
	requires (sTotal <= STEPPER_MAX_RELATIVEDISTANCE);

	behavior unknownStepper:
		assumes (stepperIndex < 0) || (stepperIndex >= STEPPER_COUNT);
		assigns \nothing;
	behavior knownStepperImmediate:
		assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
		assumes immediate != 0;

		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nA;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nC;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nD;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c7;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c8;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c9;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.initialDelayTicks;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].forward;
		assigns state[stepperIndex].cmdQueueTail, state[stepperIndex].cmdQueueHead;

		ensures state[stepperIndex].cmdQueueTail == \old(state[stepperIndex].cmdQueueHead);
		ensures state[stepperIndex].cmdQueueHead == ((\old(state[stepperIndex].cmdQueueHead) + 1) % STEPPER_COMMANDQUEUELENGTH);
		ensures state[stepperIndex].c_i == -1;
	behavior knownStepperPlanned:
		assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
		assumes immediate == 0;

		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nA;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nC;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nD;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c7;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c8;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c9;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.initialDelayTicks;
		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].forward;
		assigns state[stepperIndex].cmdQueueHead;

		ensures state[stepperIndex].cmdQueueHead == ((\old(state[stepperIndex].cmdQueueHead) + 1) % STEPPER_COMMANDQUEUELENGTH);
	disjoint behaviors;
*/
static void stepperPlanMovement_AccelerateStopToStop(int stepperIndex, double sTotal, int direction, bool immediate) {
	if((stepperIndex < 0) || (stepperIndex > 1)) { return; }
	/*@ assert (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT); */
	/*@ assert (state[stepperIndex].settings.alpha >= STEPPER_MIN_ALPHA) && (state[stepperIndex].settings.alpha <= STEPPER_MAX_ALPHA); */
	if((sTotal < STEPPER_MIN_RELATIVEDISTANCE) || (sTotal > STEPPER_MAX_RELATIVEDISTANCE)) { return; }
	/*@ assert sTotal >= STEPPER_MIN_RELATIVEDISTANCE; */
	/*@ assert sTotal <= STEPPER_MAX_RELATIVEDISTANCE; */

	const int idx = state[stepperIndex].cmdQueueHead;

	state[stepperIndex].cmdQueue[idx].cmdType = stepperCommand_AccelerateStopToStop;

	double nTotal = sTotal / state[stepperIndex].settings.alpha;

	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA = state[stepperIndex].constants.c2;
	/*@ assert state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA < (STEPPER_MAX_VMAX*STEPPER_MAX_VMAX)/(STEPPER_MIN_ACCELERATION*STEPPER_MIN_ALPHA); */
	/*@ assert state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA >= (STEPPER_MIN_VMAX*STEPPER_MIN_VMAX)/(STEPPER_MAX_ACCELERATION*STEPPER_MAX_ALPHA); */
	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD = state[stepperIndex].constants.c4;
	/*@ assert state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD < -1*(STEPPER_MAX_VMAX*STEPPER_MAX_VMAX)/(STEPPER_MIN_DECELERATION*STEPPER_MIN_ALPHA); */
	/*@ assert state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD >= -1*(STEPPER_MIN_VMAX*STEPPER_MIN_VMAX)/(STEPPER_MAX_DECELERATION*STEPPER_MAX_ALPHA); */

	double nTP = state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA + state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD;
	/*@ assert nTP >= 0; */
	/*@ assert nTP <= 4294967295; */

	/*
		Check if we have a constant velocity part (nTP < nTotal) or not (nTP >= nTotal)
	*/
	if(nTP < nTotal) {
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nC = ((uint32_t)nTotal) - nTP;
	} else {
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA = nTotal * state[stepperIndex].constants.c5;
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD = nTotal - state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA;
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nC = 0;
	}
	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.c7 = state[stepperIndex].constants.c7;
	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.c8 = state[stepperIndex].constants.c8;
	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.c9 = state[stepperIndex].constants.c9;

	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.initialDelayTicks = sqrt(state[stepperIndex].constants.c7) * (double)STEPPER_TIMERTICK_FRQ;

	state[stepperIndex].cmdQueue[idx].forward = direction;

	/*
		Make command active
	*/
	if(immediate) {
		state[stepperIndex].cmdQueueTail = state[stepperIndex].cmdQueueHead;
		state[stepperIndex].c_i = -1;
		state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
	} else {
		state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
	}

	return;
}

#ifdef ENABLE_ABSOLUTEPOSITION
	/*@
		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> (state[iStep].cmdQueueHead >= 0) && (state[iStep].cmdQueueHead < STEPPER_COMMANDQUEUELENGTH);
		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> state[iStep].constants.c2 == (state[iStep].settings.vmax * state[iStep].settings.vmax) / (2 * state[iStep].settings.acceleration * state[iStep].settings.alpha);
		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> state[iStep].constants.c4 == -1 * (state[iStep].settings.vmax * state[iStep].settings.vmax) / (2 * state[iStep].settings.deceleration * state[iStep].settings.alpha);
		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> state[iStep].constants.c5 == state[iStep].settings.deceleration / (state[iStep].settings.deceleration - state[iStep].settings.acceleration);
		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> state[iStep].constants.c7 == 2 * state[iStep].settings.alpha / state[iStep].settings.acceleration;
		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> state[iStep].constants.c8 == state[iStep].settings.acceleration / (state[iStep].settings.alpha * (double)STEPPER_TIMERTICK_FRQ * (double)STEPPER_TIMERTICK_FRQ);
		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> state[iStep].constants.c9 == state[iStep].settings.acceleration / (state[iStep].settings.alpha * (double)STEPPER_TIMERTICK_FRQ * (double)STEPPER_TIMERTICK_FRQ);
		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> state[iStep].constants.c10 == state[iStep].settings.alpha * (double)STEPPER_TIMERTICK_FRQ;

		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> (state[iStep].settings.vmax >= STEPPER_MIN_VMAX) && (state[iStep].settings.vmax <= STEPPER_MAX_VMAX);
		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> (state[iStep].settings.alpha >= STEPPER_MIN_ALPHA) && (state[iStep].settings.alpha <= STEPPER_MAX_ALPHA);
		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> (state[iStep].settings.acceleration >= STEPPER_MIN_ACCELERATION) && (state[iStep].settings.acceleration <= STEPPER_MAX_ACCELERATION);
		requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> (state[iStep].settings.deceleration >= STEPPER_MIN_DECELERATION) && (state[iStep].settings.deceleration <= STEPPER_MAX_DECELERATION);

		behavior unknownStepper:
			assumes (stepperIndex < 0) || (stepperIndex >= STEPPER_COUNT);

			assigns \nothing;
		behavior knownStepperImmediate:
			assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
			assumes immediate != 0;

			requires (state[stepperIndex].cmdQueueHead >= 0) && (state[stepperIndex].cmdQueueHead < STEPPER_COMMANDQUEUELENGTH);

			requires (state[stepperIndex].settings.vmax >= STEPPER_MIN_VMAX) && (state[stepperIndex].settings.vmax <= STEPPER_MAX_VMAX);
			requires (state[stepperIndex].settings.alpha >= STEPPER_MIN_ALPHA) && (state[stepperIndex].settings.alpha <= STEPPER_MAX_ALPHA);
			requires (state[stepperIndex].settings.acceleration >= STEPPER_MIN_ACCELERATION) && (state[stepperIndex].settings.acceleration <= STEPPER_MAX_ACCELERATION);
			requires (state[stepperIndex].settings.deceleration >= STEPPER_MIN_DECELERATION) && (state[stepperIndex].settings.deceleration <= STEPPER_MAX_DECELERATION);

			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nA;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nC;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nD;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c7;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c8;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c9;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.initialDelayTicks;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].forward;
			assigns state[stepperIndex].cmdQueueTail, state[stepperIndex].cmdQueueHead;

			ensures state[stepperIndex].cmdQueueTail == \old(state[stepperIndex].cmdQueueHead);
			ensures state[stepperIndex].cmdQueueHead == ((\old(state[stepperIndex].cmdQueueHead) + 1) % STEPPER_COMMANDQUEUELENGTH);
			ensures state[stepperIndex].c_i == -1;
		behavior knownStepperPlanned:
			assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
			assumes immediate == 0;

			requires (state[stepperIndex].cmdQueueHead >= 0) && (state[stepperIndex].cmdQueueHead < STEPPER_COMMANDQUEUELENGTH);

			requires (state[stepperIndex].settings.vmax >= STEPPER_MIN_VMAX) && (state[stepperIndex].settings.vmax <= STEPPER_MAX_VMAX);
			requires (state[stepperIndex].settings.alpha >= STEPPER_MIN_ALPHA) && (state[stepperIndex].settings.alpha <= STEPPER_MAX_ALPHA);
			requires (state[stepperIndex].settings.acceleration >= STEPPER_MIN_ACCELERATION) && (state[stepperIndex].settings.acceleration <= STEPPER_MAX_ACCELERATION);
			requires (state[stepperIndex].settings.deceleration >= STEPPER_MIN_DECELERATION) && (state[stepperIndex].settings.deceleration <= STEPPER_MAX_DECELERATION);

			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nA;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nC;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.nD;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c7;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c8;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.c9;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].data.acceleratedStopToStop.initialDelayTicks;
			assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].forward;
			assigns state[stepperIndex].cmdQueueHead;

			ensures state[stepperIndex].cmdQueueHead == ((\old(state[stepperIndex].cmdQueueHead) + 1) % STEPPER_COMMANDQUEUELENGTH);
	*/
	static void stepperPlanMovement_AccelerateStopToStopAbsolute(int stepperIndex, double sPosition, bool immediate) {
		if((stepperIndex < 0) || (stepperIndex > 1)) { return; }
		const int idx = state[stepperIndex].cmdQueueHead;

		/* Get current position */
		long int posCur = state[stepperIndex].currentPosition;

		/* Calculate target position in steps */
		long int posTarget = sPosition / state[stepperIndex].settings.alpha;
		double nTotal;
		if(posTarget > posCur) {
			nTotal = posTarget - posCur;
			state[stepperIndex].cmdQueue[idx].forward = 1;
		} else {
			nTotal = posCur - posTarget;
			state[stepperIndex].cmdQueue[idx].forward = 0;
		}

		state[stepperIndex].cmdQueue[idx].cmdType = stepperCommand_AccelerateStopToStop;
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA = state[stepperIndex].constants.c2;
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD = state[stepperIndex].constants.c4;

		uint32_t nTP = state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA + state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD;
		if(nTP < ((uint32_t)nTotal)) {
			state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nC = nTotal - ((uint32_t)nTP);
		} else {
			state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA = nTotal * state[stepperIndex].constants.c5;
			state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD = nTotal - state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA;
		}
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.c7 = state[stepperIndex].constants.c7;
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.c8 = state[stepperIndex].constants.c8;
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.c9 = state[stepperIndex].constants.c9;

		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.initialDelayTicks = sqrt(state[stepperIndex].constants.c7) * (double)STEPPER_TIMERTICK_FRQ;

		/*
			Make command active
		*/
		if(immediate) {
			state[stepperIndex].cmdQueueTail = state[stepperIndex].cmdQueueHead;
			state[stepperIndex].c_i = -1;
			state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
		} else {
			state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
		}
		return;
	}
#endif

/*@
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].settings.vmax >= STEPPER_MIN_VMAX) && (state[iStep].settings.vmax <= STEPPER_MAX_VMAX);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].cmdQueueHead >= 0) && (state[iStep].cmdQueueHead < STEPPER_COMMANDQUEUELENGTH);
	requires (immediate == true) || (immediate == false);

	behavior unknownStepper:
		assumes (stepperIndex < 0) || (stepperIndex >= STEPPER_COUNT);
		assigns \nothing;

	behavior knownStepperImmediate:
		assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
		assumes immediate == true;

		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType;
		assigns state[stepperIndex].cmdQueueTail, state[stepperIndex].cmdQueueHead;

		ensures state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType == stepperCommand_Disable;
		ensures state[stepperIndex].cmdQueueTail == \old(state[stepperIndex].cmdQueueHead);
		ensures state[stepperIndex].cmdQueueHead == ((\old(state[stepperIndex].cmdQueueHead) + 1) % STEPPER_COMMANDQUEUELENGTH);
		ensures state[stepperIndex].c_i == -1;
	behavior knownStepperPlanned:
		assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
		assumes immediate == false;

		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType;
		assigns state[stepperIndex].cmdQueueHead;

		ensures state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType == stepperCommand_Disable;
		ensures state[stepperIndex].cmdQueueHead == ((\old(state[stepperIndex].cmdQueueHead) + 1) % STEPPER_COMMANDQUEUELENGTH);
*/
static void stepperPlanMovement_Disable(int stepperIndex, bool immediate) {
	if((stepperIndex < 0) || (stepperIndex > 1)) { return; }

	const int idx = state[stepperIndex].cmdQueueHead;

	state[stepperIndex].cmdQueue[idx].cmdType = stepperCommand_Disable;

	/*
		Make command active
	*/
	if(immediate) {
		state[stepperIndex].cmdQueueTail = state[stepperIndex].cmdQueueHead;
		state[stepperIndex].c_i = -1;
		state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
	} else {
		state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
	}

	return;
}

/*@
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].cmdQueueHead >= 0) && (state[iStep].cmdQueueHead < STEPPER_COMMANDQUEUELENGTH);
	requires (immediate == true) || (immediate == false);

	behavior unknownStepper:
		assumes (stepperIndex < 0) || (stepperIndex >= STEPPER_COUNT);
		assigns \nothing;

	behavior knownStepperImmediate:
		assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
		assumes immediate == true;

		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType;
		assigns state[stepperIndex].cmdQueueTail, state[stepperIndex].cmdQueueHead;

		ensures state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType == stepperCommand_Stop;
		ensures state[stepperIndex].cmdQueueTail == \old(state[stepperIndex].cmdQueueHead);
		ensures state[stepperIndex].cmdQueueHead == ((\old(state[stepperIndex].cmdQueueHead) + 1) % STEPPER_COMMANDQUEUELENGTH);
		ensures state[stepperIndex].c_i == -1;
	behavior knownStepperPlanned:
		assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
		assumes immediate == false;

		assigns state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType;
		assigns state[stepperIndex].cmdQueueHead;

		ensures state[stepperIndex].cmdQueue[\old(state[stepperIndex].cmdQueueHead)].cmdType == stepperCommand_Stop;
		ensures state[stepperIndex].cmdQueueHead == ((\old(state[stepperIndex].cmdQueueHead) + 1) % STEPPER_COMMANDQUEUELENGTH);
*/
static void stepperPlanMovement_Stop(int stepperIndex, bool immediate) {
	if((stepperIndex < 0) || (stepperIndex > 1)) { return; }

	const int idx = state[stepperIndex].cmdQueueHead;

	state[stepperIndex].cmdQueue[idx].cmdType = stepperCommand_Stop;

	/*
		Make command active
	*/
	if(immediate) {
		state[stepperIndex].cmdQueueTail = state[stepperIndex].cmdQueueHead;
		state[stepperIndex].c_i = -1;
		state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
	} else {
		state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
	}

	return;
}

/*@
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].settings.vmax >= STEPPER_MIN_VMAX) && (state[iStep].settings.vmax <= STEPPER_MAX_VMAX);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].settings.alpha >= STEPPER_MIN_ALPHA) && (state[iStep].settings.alpha <= STEPPER_MAX_ALPHA);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> (state[iStep].settings.acceleration >= STEPPER_MIN_ACCELERATION) && (state[iStep].settings.acceleration <= STEPPER_MAX_ACCELERATION);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
	 	==> (state[iStep].settings.deceleration >= STEPPER_MIN_DECELERATION) && (state[iStep].settings.deceleration <= STEPPER_MAX_DECELERATION);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> state[iStep].constants.c7 == 2 * state[iStep].settings.alpha / state[iStep].settings.acceleration;
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
		==> state[iStep].settings.alpha * (double)STEPPER_TIMERTICK_FRQ;
	requires (immediate == true) || (immediate == false);

	behavior unknownStepper:
		assumes (stepperIndex < 0) || (stepperIndex >= STEPPER_COUNT);
		assigns \nothing;
	behavior knownStepperImmediate:
		assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
		assumes immediate == true;

		requires (v >= (state[stepperIndex].settings.alpha * STEPPER_TIMERTICK_FRQ)/(4294967295)) && (v <= state[stepperIndex].settings.alpha * STEPPER_TIMERTICK_FRQ);

	behavior knownStepperPlanned:
		assumes (stepperIndex >= 0) && (stepperIndex < STEPPER_COUNT);
		assumes immediate == false;

		requires (v >= (state[stepperIndex].settings.alpha * STEPPER_TIMERTICK_FRQ)/(4294967295)) && (v <= state[stepperIndex].settings.alpha * STEPPER_TIMERTICK_FRQ);


	disjoint behaviors;
	complete behaviors;
*/
static void stepperPlanSpeedChange(int stepperIndex, double v, int direction, bool immediate) {
	/*
		We want to change our speed from the current to v. This may
		require an acceleration or deceleration slope and will be followed
		by a constant speed movement.

		This command is normally used when the stepper driver is used for
		motion along an unknown axis (for example propelling an robot, etc.)

		The current "speed" can be determined by reading the current c_i value
		for the given stepper

		The steps are calculated the same way as with Stop-to-Stop movement
		pattern, start value is the current count. The only thing that has to
		be calculated is the number of steps that the acceleration / deceleration
		takes

		Note that if directions are NOT equal from the current or previous
		operation and the newly enqueued we have to FIRST stop the current / last
		movement and then do an acceleration to constant speed with OUR direction
	*/
	if((stepperIndex > 1) || (stepperIndex < 0)) { return; }
	const int idx = state[stepperIndex].cmdQueueHead;

	/*
		Calculate number of ticks per step at the end and fetch the current
		number of ticks. Larger number of ticks means SLOWER speed so we have
		to ACCELERATE if cTicksCurrent > cTicksEnd. We have to DECELERATE if
		cTicksCurrent < cTicksEnd
	*/
	double cTicksEnd;
	const double minTicks = sqrt(state[stepperIndex].constants.c7) * (double)STEPPER_TIMERTICK_FRQ; /* c0 for end-to-end movement */
	if(v > 0) {
		cTicksEnd = state[stepperIndex].constants.c10 / v;
	} else {
		cTicksEnd = minTicks;
	}

	double cTicksCurrent;
	int directionCurrent;

	if(state[stepperIndex].cmdQueueTail == state[stepperIndex].cmdQueueHead) {
		/* Our stepper is currently stopped */

		cTicksCurrent = minTicks;
		directionCurrent = direction;
	} else {
		if(immediate) {
			cTicksCurrent = state[stepperIndex].c_i; /* Current speed is encoded in this value */
			directionCurrent = state[stepperIndex].cmdQueue[state[stepperIndex].cmdQueueTail].forward;
		} else {
			/*
				We have to fetch the c_end from the command directly in front of us
			*/
			const int idxBefore = (idx == 0) ? STEPPER_COMMANDQUEUELENGTH-1 : idx-1;

			directionCurrent = state[stepperIndex].cmdQueue[idxBefore].forward;
			switch(state[stepperIndex].cmdQueue[idxBefore].cmdType) {
				case stepperCommand_ConstantSpeed:			cTicksCurrent = state[stepperIndex].cmdQueue[idxBefore].data.constantSpeed.cConst; break;
				case stepperCommand_AccelerateToSpeed:		cTicksCurrent = state[stepperIndex].cmdQueue[idxBefore].data.accelerateDecelerateToConstSpeed.cEnd; break;
				case stepperCommand_DecelerateToSpeed:		cTicksCurrent = state[stepperIndex].cmdQueue[idxBefore].data.accelerateDecelerateToConstSpeed.cEnd; break;
				default:									cTicksCurrent = 0; break;
			}
		}
	}

	/*
		Clamp ticks to minimum values
	*/
	if((cTicksCurrent > minTicks) || (cTicksCurrent <= 0)) {
		cTicksCurrent = minTicks;
	}
	if((cTicksEnd > minTicks) || (cTicksEnd <= 0)) {
		cTicksEnd = minTicks;
	}

	if(v == 0) { direction = directionCurrent; }

	if(directionCurrent != direction) {
		/* First we have to stop ... */
		state[stepperIndex].cmdQueue[idx].cmdType = stepperCommand_DecelerateToSpeed;
		state[stepperIndex].cmdQueue[idx].forward = directionCurrent;
		state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.c8 = state[stepperIndex].constants.c8;
		state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.c9 = state[stepperIndex].constants.c9;
		state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.cEnd = sqrt(state[stepperIndex].constants.c7) * (double)STEPPER_TIMERTICK_FRQ;
		state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.cStart = cTicksCurrent;

		state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.nA = 0;
		state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.nD = -1.0 * (state[stepperIndex].constants.c10 * state[stepperIndex].constants.c10) / (cTicksCurrent * cTicksCurrent) * state[stepperIndex].constants.c3;

		/* And then we have to accelerate into the correct direction */
		const int idx2 = (idx + 1) % STEPPER_COMMANDQUEUELENGTH;
		state[stepperIndex].cmdQueue[idx2].cmdType = stepperCommand_AccelerateToSpeed;
		state[stepperIndex].cmdQueue[idx2].forward = direction;
		state[stepperIndex].cmdQueue[idx2].data.accelerateDecelerateToConstSpeed.c8 = state[stepperIndex].constants.c8;
		state[stepperIndex].cmdQueue[idx2].data.accelerateDecelerateToConstSpeed.c9 = state[stepperIndex].constants.c9;
		state[stepperIndex].cmdQueue[idx2].data.accelerateDecelerateToConstSpeed.cStart = sqrt(state[stepperIndex].constants.c7) * (double)STEPPER_TIMERTICK_FRQ;
		state[stepperIndex].cmdQueue[idx2].data.accelerateDecelerateToConstSpeed.cEnd = cTicksEnd;

		state[stepperIndex].cmdQueue[idx2].data.accelerateDecelerateToConstSpeed.nA = v*v * state[stepperIndex].constants.c1;
		state[stepperIndex].cmdQueue[idx2].data.accelerateDecelerateToConstSpeed.nD = 0;

		/*
			Make command active
		*/
		if(immediate) {
			state[stepperIndex].cmdQueueTail = state[stepperIndex].cmdQueueHead;
			state[stepperIndex].c_i = -1;
		}
		state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 2) % STEPPER_COMMANDQUEUELENGTH;
	} else {
		state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.c8 = state[stepperIndex].constants.c8;
		state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.c9 = state[stepperIndex].constants.c9;
		state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.cEnd = cTicksEnd;
		state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.cStart = cTicksCurrent;
		state[stepperIndex].cmdQueue[idx].forward = direction;

		if(cTicksEnd < cTicksCurrent) {
			/*
				Accelerate
			*/
			double nA = (v*v - (state[stepperIndex].constants.c10 * state[stepperIndex].constants.c10) / (cTicksCurrent * cTicksCurrent)) * state[stepperIndex].constants.c1;

			state[stepperIndex].cmdQueue[idx].cmdType = stepperCommand_AccelerateToSpeed;
			state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.nA = nA;
			state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.nD = 0;
		} else if(cTicksEnd > cTicksCurrent) {
			/*
				Decelerate
			*/
			double nD = (v * v - (state[stepperIndex].constants.c10 * state[stepperIndex].constants.c10) / (cTicksCurrent * cTicksCurrent)) * state[stepperIndex].constants.c3;

			state[stepperIndex].cmdQueue[idx].cmdType = stepperCommand_DecelerateToSpeed;
			state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.nA = 0;
			state[stepperIndex].cmdQueue[idx].data.accelerateDecelerateToConstSpeed.nD = nD;
		}

		/*
			Make command active
		*/
		if(immediate) {
			state[stepperIndex].cmdQueueTail = state[stepperIndex].cmdQueueHead;
			state[stepperIndex].c_i = -1;
		}
		state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
	}
}

/*
	==============================
	= I2C communiation subsystem =
	==============================
*/

enum i2cCommand {
	i2cCmd_GetAccelerateDecelerate		= 0x01,	/* 8 byte payload Slave -> Master + 1 Byte Status */
	i2cCmd_SetAccelerateDecelerate		= 0x02,	/* 8 Byte payload Master -> Slave */
	i2cCmd_GetVMax						= 0x03, /* 4 byte payload Slave -> Master + 1 Byte Status */
	i2cCmd_SetVMax						= 0x04,	/* 4 Byte payload Mater -> Slave */
	i2cCmd_GetAlpha						= 0x05,	/* 4 Byte payload Slave -> Master + 1 Byte Status */
	i2cCmd_SetAlpha						= 0x06,	/* 4 Byte payload Master -> Slave */
	i2cCmd_GetMicrostepping				= 0x07, /* 1 Byte payload Slave -> Master (2x 3 Bit) + 1 Byte Status */
	i2cCmd_SetMicrostepping				= 0x08, /* 1 Byte payload Master -> Slave */
	i2cCmd_SetAbsolutePosition			= 0x09, /* 4 Byte payload Master -> Slave */
	i2cCmd_GetAbsolutePosition			= 0x0A, /* 4 Byte payload Slave -> Master */
	i2cCmd_GetFault						= 0x0E, /* 1 Byte payload Master -> Slave (IS status) */
	i2cCmd_RecalculateConstants			= 0x0F, /* Used to trigger recalculation of all constants (expensive operation; system should be stopped) */



	i2cCmd_GetCommandQueueSize			= 0x10,	/* Get size (first byte) and unused entires (second byte) of command queue + 1 Byte status */



	i2cCmd_Queue_Sync					= 0x20,	/* Sync. point; 1 Byte Channel */
	i2cCmd_Queue_ConstSpeed				= 0x21,	/* Constant speed; 1 Byte Channel; 4 Byte Speed */
	i2cCmd_Queue_MoveTo					= 0x22,	/* Move To (accelerated); 1 Byte Channel; 4 Byte Position */
	i2cCmd_Queue_ConstSpeedAccel		= 0x23,	/* Constant speed with acceleration/deceleration; 1 Byte channel; 4 Byte speed */
	i2cCmd_Queue_MoveToAbsolute			= 0x24, /* Accelerated move to absolute position (passed in angular position) */
	i2cCmd_Queue_Hold					= 0x2E,	/* Hold position; 1 byte channel */
	i2cCmd_Queue_DisableDrv				= 0x2F,	/* Disable drivers; 1 byte channel (both have to be ordered to disable to be effective) */

	i2cCmd_Exec_Sync					= 0x30,	/* Sync. point; 1 Byte Channel */
	i2cCmd_Exec_ConstSpeed				= 0x31,	/* Constant speed; 1 Byte Channel; 4 Byte Speed */
	i2cCmd_Exec_MoveTo					= 0x32,	/* Move To (accelerated); 1 Byte Channel; 4 Byte Position */
	i2cCmd_Exec_ConstSpeedAccel			= 0x33,	/* Constant speed with acceleration/deceleration; 1 Byte channel; 4 Byte speed */
	i2cCmd_Exec_MoveToAbsolute			= 0x34, /* Accelerated move to absolute position (passed in angular position) */
	i2cCmd_Exec_Hold					= 0x3E,	/* Hold position; 1 byte channel */
	i2cCmd_Exec_DisableDrv				= 0x3F,	/* Disable drivers; 1 byte channel (both have to be ordered to disable to be effective) */

	i2cCmd_EmergencyStop				= 0xFE,	/* Keeps motors engaged but stopped */
	i2cCmd_EmergencyOff					= 0xFF, /* Keeps motors disabled */
};

#ifndef STEPPER_I2C_BUFFERSIZE_RX
	#define STEPPER_I2C_BUFFERSIZE_RX	128
#endif
#ifndef STEPPER_I2C_BUFFERSIZE_TX
	#define STEPPER_I2C_BUFFERSIZE_TX	64
#endif

static volatile uint8_t i2cBuffer_RX[STEPPER_I2C_BUFFERSIZE_RX];
static volatile int i2cBuffer_RX_Head = 0;
static volatile int i2cBuffer_RX_Tail = 0;

static volatile uint8_t i2cBuffer_TX[STEPPER_I2C_BUFFERSIZE_TX];
static volatile int i2cBuffer_TX_Head = 0;
static volatile int i2cBuffer_TX_Tail = 0;

/*
	The receive and transmit handlers only enqueue
	the data into the receive queue or transmit from
	the TX queue to minimize jitter for TIMER2. The
	messages are handeled from the main loop which is
	always interruptable by the interrupt handlers.
*/
/*@
	requires i2cBuffer_RX_Head >= 0;
	requires i2cBuffer_RX_Head < STEPPER_I2C_BUFFERSIZE_RX;

	behavior bufferOverflow:
		assumes (i2cBuffer_RX_Head + 1) % STEPPER_I2C_BUFFERSIZE_RX == i2cBuffer_RX_Tail;

		assigns \nothing;
	behavior bufferAvail:
		assumes (i2cBuffer_RX_Head + 1) % STEPPER_I2C_BUFFERSIZE_RX != i2cBuffer_RX_Tail;

		assigns i2cBuffer_RX_Head;
		assigns i2cBuffer_RX[i2cBuffer_RX_Head];

		ensures i2cBuffer_RX_Head >= 0;
		ensures i2cBuffer_RX_Head < STEPPER_I2C_BUFFERSIZE_RX;
		ensures i2cBuffer_RX_Head == \old((i2cBuffer_RX_Head + 1) % STEPPER_I2C_BUFFERSIZE_RX);
	disjoint behaviors;
	complete behaviors;
*/
static inline void i2cEventReceived(uint8_t data) {
	// Do whatever we want with the received data
	if(((i2cBuffer_RX_Head + 1) % STEPPER_I2C_BUFFERSIZE_RX) == i2cBuffer_RX_Tail) {
		// Buffer overflow. ToDo
		return;
	}
	i2cBuffer_RX[i2cBuffer_RX_Head] = data;
	i2cBuffer_RX_Head = (i2cBuffer_RX_Head + 1) % STEPPER_I2C_BUFFERSIZE_RX;
}
/*@
	assigns \nothing;
*/
static inline void i2cEventBusError() {
	// Currently we force a reset by using the watchdog after 1s delay
	return;
}
/*@
	requires i2cBuffer_TX_Tail >= 0;
	requires i2cBuffer_TX_Tail < STEPPER_I2C_BUFFERSIZE_TX;

	behavior bufferUnderrun:
		assumes i2cBuffer_TX_Head == i2cBuffer_TX_Tail;

		assigns \nothing;
	behavior bufferDefault:
		assumes i2cBuffer_TX_Head != i2cBuffer_TX_Tail;

		assigns i2cBuffer_TX_Tail;
		assigns i2cBuffer_TX[i2cBuffer_TX_Tail];

		ensures i2cBuffer_TX_Tail >= 0;
		ensures i2cBuffer_TX_Tail < STEPPER_I2C_BUFFERSIZE_TX;
		ensures i2cBuffer_TX_Tail == (\old(i2cBuffer_TX_Tail) + 1) % STEPPER_I2C_BUFFERSIZE_TX;
	complete behaviors;
	disjoint behaviors;
*/
static inline uint8_t i2cEventTransmit() {
	if(i2cBuffer_TX_Head == i2cBuffer_TX_Tail) {
		/* Empty buffer - buffer underrun ... ToDo */
		return 0x00;
	} else {
		uint8_t r = i2cBuffer_TX[i2cBuffer_TX_Tail];
		i2cBuffer_TX_Tail = (i2cBuffer_TX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_TX;
		return r;
	}
}

/*@
	requires \valid(&SREG) && \valid(&TWAR) && \valid(&TWCR);

	assigns SREG;
	assigns TWAR, TWCR;

	ensures TWCR == 0xC5;
	ensures (TWAR == address << 1) || (TWAR == ((address << 1) | 0x01));
*/
static void i2cSlaveInit(uint8_t address) {
	#ifndef FRAMAC_SKIP
		cli();
	#endif

	TWAR = (address << 1) | 0x00; // Respond to general calls and calls towards us
	TWCR = 0xC5; // Set TWIE (TWI Interrupt enable), TWEN (TWI Enable), TWEA (TWI Enable Acknowledgement), TWINT (Clear TWINT flag by writing a 1)

	#ifndef FRAMAC_SKIP
		sei();
	#endif
	return;
}

/*@
	requires \valid_read(&TWSR) && \valid_read(&TWDR) && \valid(&TWDR) && \valid(&TWCR);

	ensures TWCR == 0xC5;
*/
ISR(TWI_vect) {
	switch(TW_STATUS) { /* Note: TW_STATUS is an macro that masks status bits from TWSR) */
		case TW_SR_SLA_ACK:
			/*
				Slave will read, slave has been addresses and address
				has been acknowledged
			*/
			break;
		case TW_SR_DATA_ACK:
			/*
				We have received data. This is now contained in the TWI
				data register (TWDR)
			*/
			i2cEventReceived(TWDR);
			break;
		case TW_ST_SLA_ACK:
		case TW_ST_DATA_ACK:
			/*
				Either slave selected (SLA_ACK) and data requested or data
				transmitted, ACK received and next data requested
			*/
			TWDR = i2cEventTransmit();
			break;
		case TW_BUS_ERROR:
			i2cEventBusError();
			break;
		default:
			break;
	}
	TWCR = 0xC5; // Set TWIE (TWI Interrupt enable), TWEN (TWI Enable), TWEA (TWI Enable Acknowledgement), TWINT (Clear TWINT flag by writing a 1)
}

/*@
	requires (i2cBuffer_TX_Head >= 0) && (i2cBuffer_TX_Head < STEPPER_I2C_BUFFERSIZE_TX);
	requires ((i2cBuffer_TX_Head >= i2cBuffer_TX_Tail) && ((STEPPER_I2C_BUFFERSIZE_TX - (i2cBuffer_TX_Head - i2cBuffer_TX_Tail)) >= 4))
		|| ((i2cBuffer_TX_Head < i2cBuffer_TX_Tail) && (STEPPER_I2C_BUFFERSIZE_TX - i2cBuffer_TX_Head - (STEPPER_I2C_BUFFERSIZE_TX - i2cBuffer_TX_Tail)) >= 4);
	assigns i2cBuffer_TX[i2cBuffer_TX_Head];
	assigns i2cBuffer_TX[(i2cBuffer_TX_Head+1) % STEPPER_I2C_BUFFERSIZE_TX];
	assigns i2cBuffer_TX[(i2cBuffer_TX_Head+2) % STEPPER_I2C_BUFFERSIZE_TX];
	assigns i2cBuffer_TX[(i2cBuffer_TX_Head+3) % STEPPER_I2C_BUFFERSIZE_TX];
	assigns i2cBuffer_TX[(i2cBuffer_TX_Head+4) % STEPPER_I2C_BUFFERSIZE_TX];
	assigns i2cBuffer_TX_Head;

	ensures i2cBuffer_TX_Head == (\old(i2cBuffer_TX_Head + 4) % STEPPER_I2C_BUFFERSIZE_TX);
*/
static inline void i2cTXDouble(double f) {
	/*
		Note this is REALLY a hackish and unclean
		way to do ... this is NOT standards conformant
		C since C does not really require IEEE floating
		point values. One SHOULD do a real serialization
		instead but that would take WAY more cycles
	*/
	union {
		double f;
		uint8_t b[4];
	} d;

	d.f = f;
	i2cBuffer_TX[i2cBuffer_TX_Head] = d.b[0];
	i2cBuffer_TX[(i2cBuffer_TX_Head+1) % STEPPER_I2C_BUFFERSIZE_TX] = d.b[1];
	i2cBuffer_TX[(i2cBuffer_TX_Head+2) % STEPPER_I2C_BUFFERSIZE_TX] = d.b[2];
	i2cBuffer_TX[(i2cBuffer_TX_Head+3) % STEPPER_I2C_BUFFERSIZE_TX] = d.b[3];
	i2cBuffer_TX_Head = (i2cBuffer_TX_Head+4) % STEPPER_I2C_BUFFERSIZE_TX;
	return;
}

/*@
	requires \valid(&i2cBuffer_RX[i2cBuffer_RX_Tail]);
	requires \valid(&i2cBuffer_RX[(i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX]);
        requires \valid(&i2cBuffer_RX[(i2cBuffer_RX_Tail + 2) % STEPPER_I2C_BUFFERSIZE_RX]);
        requires \valid(&i2cBuffer_RX[(i2cBuffer_RX_Tail + 3) % STEPPER_I2C_BUFFERSIZE_RX]);
	requires i2cBuffer_RX_Tail >= 0;
	requires i2cBuffer_RX_Tail < STEPPER_I2C_BUFFERSIZE_RX;

	assigns i2cBuffer_RX_Tail;

	ensures i2cBuffer_RX_Tail == ((\old(i2cBuffer_RX_Tail) + 4) % STEPPER_I2C_BUFFERSIZE_RX);
*/
static inline double i2cRXDouble() {
	/*
		Note this is REALLY a hackish and unclean
		way to do ... this is NOT standards conformant
		C since C does not really require IEEE floating
		point values. One SHOULD do a real deserialization
		instead but that would take WAY more cycles
	*/
	union {
		double f;
		uint8_t b[4];
	} d;

	d.b[0] = i2cBuffer_RX[i2cBuffer_RX_Tail];
	i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
	d.b[1] = i2cBuffer_RX[i2cBuffer_RX_Tail];
	i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
	d.b[2] = i2cBuffer_RX[i2cBuffer_RX_Tail];
	i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
	d.b[3] = i2cBuffer_RX[i2cBuffer_RX_Tail];
	i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;

	return d.f;
}

/*@
	requires i2cBuffer_TX_Tail >= 0;
	requires i2cBuffer_TX_Tail < STEPPER_I2C_BUFFERSIZE_TX;
	requires i2cBuffer_TX_Head >= 0;
	requires i2cBuffer_TX_Head < STEPPER_I2C_BUFFERSIZE_TX;
	requires i2cBuffer_RX_Tail >= 0;
	requires i2cBuffer_RX_Tail < STEPPER_I2C_BUFFERSIZE_RX;
	requires i2cBuffer_RX_Head >= 0;
	requires i2cBuffer_RX_Head < STEPPER_I2C_BUFFERSIZE_RX;

	requires \valid(&SREG);

	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
	 			==> (state[iStep].settings.vmax >= STEPPER_MIN_VMAX) && (state[iStep].settings.vmax <= STEPPER_MAX_VMAX);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> (state[iStep].settings.alpha >= STEPPER_MIN_ALPHA) && (state[iStep].settings.alpha <= STEPPER_MAX_ALPHA);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
			 	==> (state[iStep].settings.acceleration >= STEPPER_MIN_ACCELERATION) && (state[iStep].settings.acceleration <= STEPPER_MAX_ACCELERATION);
	requires \forall integer iStep; 0 <= iStep < STEPPER_COUNT
				==> (state[iStep].settings.deceleration >= STEPPER_MIN_DECELERATION) && (state[iStep].settings.deceleration <= STEPPER_MAX_DECELERATION);


	assigns SREG;

	assigns	i2cBuffer_TX_Tail, i2cBuffer_TX_Head;
	assigns i2cBuffer_RX_Tail, i2cBuffer_RX_Head;

	ensures i2cBuffer_TX_Tail >= 0;
	ensures i2cBuffer_TX_Tail < STEPPER_I2C_BUFFERSIZE_TX;
	ensures i2cBuffer_TX_Head >= 0;
	ensures i2cBuffer_TX_Head < STEPPER_I2C_BUFFERSIZE_TX;
	ensures i2cBuffer_RX_Tail >= 0;
	ensures i2cBuffer_RX_Tail < STEPPER_I2C_BUFFERSIZE_RX;
	ensures i2cBuffer_RX_Head >= 0;
	ensures i2cBuffer_RX_Head < STEPPER_I2C_BUFFERSIZE_RX;

	behavior cmdSetMax_ValidStep:
		assumes ((i2cBuffer_RX_Tail <= i2cBuffer_RX_Head) ? (i2cBuffer_RX_Head - i2cBuffer_RX_Tail) : (STEPPER_I2C_BUFFERSIZE_RX - i2cBuffer_RX_Tail + i2cBuffer_RX_Head)) >= 6;
		assumes i2cBuffer_RX[i2cBuffer_RX_Tail] == i2cCmd_SetVMax;
		assumes (i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX] >= 0) && (i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX] < STEPPER_COUNT);

		assigns state[i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX]].settings.vmax;

		ensures (state[i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX]].settings.vmax >= STEPPER_MIN_VMAX) && (state[i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX]].settings.vmax < STEPPER_MAX_VMAX);
	behavior cmdSetAlpha_ValidStep:
		assumes ((i2cBuffer_RX_Tail <= i2cBuffer_RX_Head) ? (i2cBuffer_RX_Head - i2cBuffer_RX_Tail) : (STEPPER_I2C_BUFFERSIZE_RX - i2cBuffer_RX_Tail + i2cBuffer_RX_Head)) >= 6;
		assumes i2cBuffer_RX[i2cBuffer_RX_Tail] == i2cCmd_SetAlpha;
		assumes (i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX] >= 0) && (i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX] < STEPPER_COUNT);

		assigns state[i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX]].settings.alpha;

		ensures (state[i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX]].settings.alpha >= STEPPER_MIN_ALPHA) && (state[i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX]].settings.alpha < STEPPER_MAX_ALPHA);
	behavior cmdSetAcceleration:
		assumes ((i2cBuffer_RX_Tail <= i2cBuffer_RX_Head) ? (i2cBuffer_RX_Head - i2cBuffer_RX_Tail) : (STEPPER_I2C_BUFFERSIZE_RX - i2cBuffer_RX_Tail + i2cBuffer_RX_Head)) >= 10;
		assumes i2cBuffer_RX[i2cBuffer_RX_Tail] == i2cCmd_SetAccelerateDecelerate;
		assumes (i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX] >= 0) && (i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX] < STEPPER_COUNT);

		ensures \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> (state[iStep].settings.acceleration >= STEPPER_MIN_ACCELERATION) && (state[iStep].settings.acceleration <= STEPPER_MAX_ACCELERATION);
		ensures \forall integer iStep; 0 <= iStep < STEPPER_COUNT
					==> (state[iStep].settings.deceleration >= STEPPER_MIN_DECELERATION) && (state[iStep].settings.deceleration <= STEPPER_MAX_DECELERATION);

	disjoint behaviors;
*/
/*
*/
static void i2cMessageLoop() {
	uint8_t rcvBytes = (i2cBuffer_RX_Tail <= i2cBuffer_RX_Head) ? (i2cBuffer_RX_Head - i2cBuffer_RX_Tail) : (STEPPER_I2C_BUFFERSIZE_RX - i2cBuffer_RX_Tail + i2cBuffer_RX_Head);

	if(rcvBytes == 0) {
		return; /* Nothing received */
	}
	uint8_t cmd = i2cBuffer_RX[i2cBuffer_RX_Tail];
	uint8_t txAvail = STEPPER_I2C_BUFFERSIZE_TX - ((i2cBuffer_TX_Tail <= i2cBuffer_TX_Head) ? (i2cBuffer_TX_Head - i2cBuffer_TX_Tail) : (STEPPER_I2C_BUFFERSIZE_TX - i2cBuffer_TX_Tail + i2cBuffer_TX_Head));
	/*@ assert (txAvail <= STEPPER_I2C_BUFFERSIZE_TX) && (txAvail >= 0); */
	/*@ assert (rcvBytes <= STEPPER_I2C_BUFFERSIZE_RX) && (rcvBytes >= 0); */

	/*
		We use a switch statement here. Would a jump table
		be possible and more efficient?
	*/
	switch(cmd) {
		case i2cCmd_GetAccelerateDecelerate:
			{
				/*
					Two byte command that allows reading of 8 bytes representing
					currently configured acceleration and deceleration on the
					selected channel
				*/
				if(rcvBytes < 2) {
					return; /* Command not fully received until now */
				}
				if(txAvail < 8) {
					/*
						Command is currently NOT satisfyable, simply wait
						till more I2C data has been transmitted and reprocess
						this command in a tight loop
					*/
					return;
				}
				uint8_t channel = i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX];

				/* Done */
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 2) % STEPPER_I2C_BUFFERSIZE_RX; /* Discard command in RX buffer */

				/* Build our response */
				double a = (channel < 2) ? state[channel].settings.acceleration : 0.0;
				double d = (channel < 2) ? state[channel].settings.deceleration : 0.0;

				/* Encode IEEE float to 4 byte sequence in little endian */
				i2cTXDouble(a);
				i2cTXDouble(d);
			}
			break;
		case i2cCmd_SetAccelerateDecelerate:
			{
				if(rcvBytes < 2+4+4) {
					return; /* Command not fully received */
				}
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				double a = i2cRXDouble();
				double d = i2cRXDouble();

				if((a >= STEPPER_MIN_ACCELERATION) && (d > STEPPER_MIN_DECELERATION) && (a < STEPPER_MAX_ACCELERATION) && (d <= STEPPER_MAX_DECELERATION)) {
					if(channel < 2) {
						state[channel].settings.acceleration = a;
						state[channel].settings.deceleration = d;
					}
				}
				/* Done */
			}
			break;
		case i2cCmd_GetVMax:
			{
				/*
					Two byte command that allows reading of 4 bytes representing currently configured
					maximum speed of selected channel
					*/
				if(rcvBytes < 2) {
					return; /* Command not fully received until now */
				}
				if(txAvail < 4) {
					/*
						Command is currently NOT satisfyable, simply wait
						till more I2C data has been transmitted and reprocess
						this command in a tight loop
					*/
					return;
				}
				uint8_t channel = i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX];
				double vmax = (channel < 2) ? state[channel].settings.vmax : -1.0;

				/* Encode IEEE float to 4 byte sequence in little endian */
				i2cTXDouble(vmax);

				/* Done */
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 2) % STEPPER_I2C_BUFFERSIZE_RX; /* Discard command in RX buffer */
			}
			break;
		case i2cCmd_SetVMax:
			{
				if(rcvBytes < 2+4) {
					return; /* Command not fully received */
				}
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				double vmax = i2cRXDouble();

				if((vmax >= STEPPER_MIN_VMAX) && (vmax < STEPPER_MAX_VMAX)) {
					if(channel < 2) {
						state[channel].settings.vmax = vmax;
					}
				}
				/* Done */
			}
			break;
		case i2cCmd_GetAlpha:
			{
				/*
					Two byte command that allows reading of 4 bytes representing currently configured
					alpha value of selected channel
				*/
				if(rcvBytes < 2) {
					return; /* Command not fully received until now */
				}
				if(txAvail < 4) {
					/*
						Command is currently NOT satisfyable, simply wait
						till more I2C data has been transmitted and reprocess
						this command in a tight loop
					*/
					return;
				}
				uint8_t channel = i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX];
				double alpha = (channel < 2) ? state[channel].settings.alpha : -1.0;

				/* Encode IEEE float to 4 byte sequence in little endian */
				i2cTXDouble(alpha);

				/* Done */
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 2) % STEPPER_I2C_BUFFERSIZE_RX; /* Discard command in RX buffer */
			}
			break;
		case i2cCmd_SetAlpha:
			{
				if(rcvBytes < 2+4) {
					return; /* Command not fully received */
				}
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				double alpha = i2cRXDouble();

				if((alpha >= STEPPER_MIN_ALPHA) && (alpha < STEPPER_MAX_ALPHA)) {
					if(channel < 2) {
						state[channel].settings.alpha = alpha;
					}
				}
				/* Done */
			}
			break;
		case i2cCmd_GetMicrostepping:
			{
				if(txAvail < 1) {
					/*
						Command is currently NOT satisfyable, simply wait
						till more I2C data has been transmitted and reprocess
						this command in a tight loop
					*/
					return;
				}
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX; /* Discard command in RX buffer */

				i2cBuffer_TX[i2cBuffer_TX_Head] = stateMicrostepping;
				i2cBuffer_TX_Head = (i2cBuffer_TX_Head + 1) % STEPPER_I2C_BUFFERSIZE_TX;
			}
			break;
		case i2cCmd_SetMicrostepping:
			{
				if(rcvBytes < 2) {
					return; /* Command not fully received until now */
				}
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t microstepNew = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				stepperSetMicrostepping(microstepNew);
			}
			break;
		#ifdef ENABLE_ABSOLUTEPOSITION
			case i2cCmd_SetAbsolutePosition:
				{
					if(rcvBytes < 2+4) {
						return; /* Command not fully received */
					}
					i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
					uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
					i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
					double absPosition = i2cRXDouble();

					if(channel < 2) {
						state[channel].currentPosition = absPosition / state[channel].settings.alpha;
					}
					/* Done */
				}
				break;
			case i2cCmd_GetAbsolutePosition:
				{
					/*
						Two byte command that allows reading of 4 bytes representing currently configured
						alpha value of selected channel
					*/
					if(rcvBytes < 2) {
						return; /* Command not fully received until now */
					}
					if(txAvail < 4) {
						/*
							Command is currently NOT satisfyable, simply wait
							till more I2C data has been transmitted and reprocess
							this command in a tight loop
						*/
						return;
					}
					uint8_t channel = i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX];
					double curPos = (channel < 2) ? state[channel].currentPosition * state[channel].settings.alpha : 1.0e38;

					/* Encode IEEE float to 4 byte sequence in little endian */
					i2cTXDouble(curPos);

					/* Done */
					i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 2) % STEPPER_I2C_BUFFERSIZE_RX; /* Discard command in RX buffer */
				}
				break;
		#endif
		case i2cCmd_GetFault:
			{
				if(txAvail < 1) {
					/*
						Command is currently NOT satisfyable, simply wait
						till more I2C data has been transmitted and reprocess
						this command in a tight loop
					*/
					return;
				}
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX; /* Discard command in RX buffer */

				/*
					Fetch fault pins with interrupts disabled to prevent
					race between reading and resetting of fault state pins.
				*/
				#ifndef FRAMAC_SKIP
					cli();
				#endif
				i2cBuffer_TX[i2cBuffer_TX_Head] = stateFault;
				stateFault = 0; // Reset fault pins after readout
				#ifndef FRAMAC_SKIP
					sei();
				#endif

				i2cBuffer_TX_Head = (i2cBuffer_TX_Head + 1) % STEPPER_I2C_BUFFERSIZE_RX;
			}
			break;
		case i2cCmd_RecalculateConstants:
			{
				if(rcvBytes < 2) {
					break; /* Command not fully received until now */
				}
				uint8_t channel = i2cBuffer_RX[(i2cBuffer_RX_Tail+1) % STEPPER_I2C_BUFFERSIZE_RX];
				updateConstants(channel);
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 2) % STEPPER_I2C_BUFFERSIZE_RX; /* Discard command in RX buffer */
			}
			break;
		case i2cCmd_GetCommandQueueSize:
			{
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX; /* Discard command in RX buffer */
				if(txAvail < 2) {
					/*
						Command is currently NOT satisfyable, simply wait
						till more I2C data has been transmitted and reprocess
						this command in a tight loop
					*/
					return;
				} else {
					i2cBuffer_TX[i2cBuffer_TX_Head] = STEPPER_COMMANDQUEUELENGTH;
					i2cBuffer_TX_Head = (i2cBuffer_TX_Head + 1) % STEPPER_I2C_BUFFERSIZE_TX;
					i2cBuffer_TX[i2cBuffer_TX_Head] = 0; /* TODO!!!! */
					i2cBuffer_TX_Head = (i2cBuffer_TX_Head + 1) % STEPPER_I2C_BUFFERSIZE_TX;
				}
			}
			break;
		case i2cCmd_Queue_Sync:
			/*
				TODO: Implement synchronization

				Synchronization allows all stepepr boards inside a larger system
				to reach a synchronization point and then trigger movement processing
				simultanousely via general call.
			*/
			i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 2) % STEPPER_I2C_BUFFERSIZE_RX; /* Discard command in RX buffer */
			break;
		case i2cCmd_Queue_ConstSpeed:
			{
				if(rcvBytes < 2+4) {
					return; /* Command not fully received */
				}

				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				double constSpeed = i2cRXDouble();

				if(channel >= 2) {
					return; /* Ignore non existing channels */
				}

				if(constSpeed < 0) {
					stepperPlanMovement_ConstantSpeed(channel, -1.0*constSpeed, 0, false);
				} else {
					stepperPlanMovement_ConstantSpeed(channel, constSpeed, 1, false);
				}

				/* Done */
			}
			break;
		case i2cCmd_Queue_MoveTo:
			{
				if(rcvBytes < 2+4) {
					return; /* Command not fully received */
				}

				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				double stepAccelDecel = i2cRXDouble();

				if(channel >= 2) {
					return; /* Ignore non existing channels */
				}

				if(stepAccelDecel < 0) {
					stepperPlanMovement_AccelerateStopToStop(channel, -1.0*stepAccelDecel, 0, false);
				} else {
					stepperPlanMovement_AccelerateStopToStop(channel, stepAccelDecel, 1, false);
				}
				/* Done */
			}
			break;
		#ifdef ENABLE_ABSOLUTEPOSITION
			case i2cCmd_Queue_MoveToAbsolute:
				{
					if(rcvBytes < 2+4) {
						return; /* Command not fully received */
					}

					i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
					uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
					i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
					double stepAccelDecel = i2cRXDouble();

					if(channel >= 2) {
						return; /* Ignore non existing channels */
					}

					stepperPlanMovement_AccelerateStopToStopAbsolute(channel, stepAccelDecel, false);
					/* Done */
				}
				break;
		#endif
		case i2cCmd_Queue_ConstSpeedAccel:
			{
				if(rcvBytes < 2+4) {
					return; /* Command not fully received */
				}
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				double constSpeed = i2cRXDouble();

				if(channel >= 2) {
					return; /* Ignore non existing channels */
				}

				if(constSpeed < 0) {
					stepperPlanSpeedChange(channel, -1.0*constSpeed, 0, false);
				} else {
					stepperPlanSpeedChange(channel, constSpeed, 1, false);
				}

				/* Done */
			}
			break;
		case i2cCmd_Queue_Hold:
			{
				if(rcvBytes < 2) {
					return; /* Not fully received */
				}
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				stepperPlanMovement_Stop(channel, false);
			}
			break;
		case i2cCmd_Queue_DisableDrv:
			{
				if(rcvBytes < 2) {
					return; /* Not fully received */
				}
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				break;
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				stepperPlanMovement_Disable(channel, false);
			}
		case i2cCmd_Exec_Sync:
			/*
				TODO: Implement synchronization

				Synchronization allows all stepepr boards inside a larger system
				to reach a synchronization point and then trigger movement processing
				simultanousely via general call.
			*/
			i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 2) % STEPPER_I2C_BUFFERSIZE_RX; /* Discard command in RX buffer */
			break;
		case i2cCmd_Exec_ConstSpeed:
			{
				if(rcvBytes < 2+4) {
					return; /* Command not fully received */
				}

				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				double constSpeed = i2cRXDouble();

				if(channel >= 2) {
					return; /* Ignore non existing channels */
				}

				if(constSpeed < 0) {
					stepperPlanMovement_ConstantSpeed(channel, -1.0*constSpeed, 0, true);
				} else {
					stepperPlanMovement_ConstantSpeed(channel, constSpeed, 1, true);
				}

				/* Done */
			}
			break;
		case i2cCmd_Exec_MoveTo:
			{
				if(rcvBytes < 2+4) {
					return; /* Command not fully received */
				}

				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				double stepAccelDecel = i2cRXDouble();

				if(channel >= 2) {
					return; /* Ignore non existing channels */
				}

				if(stepAccelDecel < 0) {
					stepperPlanMovement_AccelerateStopToStop(channel, -1.0*stepAccelDecel, 0, true);
				} else {
					stepperPlanMovement_AccelerateStopToStop(channel, stepAccelDecel, 1, true);
				}

				/* Done */
			}
			break;
		case i2cCmd_Exec_ConstSpeedAccel:
			{
				if(rcvBytes < 2+4) {
					return; /* Command not fully received */
				}

				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				double constSpeed = i2cRXDouble();

				if(channel >= 2) {
					return; /* Ignore non existing channels */
				}

				if(constSpeed < 0) {
					stepperPlanSpeedChange(channel, -1.0*constSpeed, 0, true);
				} else {
					stepperPlanSpeedChange(channel, constSpeed, 1, true);
				}

				/* Done */
			}
			break;
		#ifdef ENABLE_ABSOLUTEPOSITION
			case i2cCmd_Exec_MoveToAbsolute:
				{
					if(rcvBytes < 2+4) {
						return; /* Command not fully received */
					}

					i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
					uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
					i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
					double stepAccelDecel = i2cRXDouble();

					if(channel >= 2) {
						return; /* Ignore non existing channels */
					}

					stepperPlanMovement_AccelerateStopToStopAbsolute(channel, stepAccelDecel, false);
					/* Done */
				}
				break;
		#endif
		case i2cCmd_Exec_Hold:
			{
				if(rcvBytes < 2) {
					return; /* Not fully received */
				}
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				stepperPlanMovement_Stop(channel, true);
			}
			break;
		case i2cCmd_Exec_DisableDrv:
			{
				if(rcvBytes < 2) {
					return; /* Not fully received */
				}
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				uint8_t channel = i2cBuffer_RX[i2cBuffer_RX_Tail];
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				stepperPlanMovement_Disable(channel, true);
			}
			break;
		case i2cCmd_EmergencyStop:
			{
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				stepperPlanMovement_Stop(0, true);
				stepperPlanMovement_Stop(1, true);
			}
			break;
		case i2cCmd_EmergencyOff:
			{
				i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
				stepperPlanMovement_Disable(0, true);
				stepperPlanMovement_Disable(1, true);
			}
			break;
		default:
			/*
				Unknown command. Simply ignore this byte. This allows
				the master to re-synchronize by delaying some arbitrary
				time and __slowly__ sending a sequence of zeros
			*/
			i2cBuffer_RX_Tail = (i2cBuffer_RX_Tail + 1) % STEPPER_I2C_BUFFERSIZE_RX;
			break;
	}
}

/*
	================================================
	= Main initialization and systick utility code =
	================================================
*/

/*
	System tick timer
*/

#define SYSCLK_TIMER_OVERFLOW_MICROS	(64L * 256L * (F_CPU / 1000000L))
#define SYSCLK_MILLI_INCREMENT			(SYSCLK_TIMER_OVERFLOW_MICROS / 1000)
#define SYSCLK_MILLIFRACT_INCREMENT		((SYSCLK_TIMER_OVERFLOW_MICROS % 1000) >> 3)
#define SYSCLK_MILLIFRACT_MAXIMUM		(1000 >> 3)

volatile unsigned long int systemMillis					= 0;
volatile unsigned long int systemMilliFractional		= 0;
volatile unsigned long int systemMonotonicOverflowCnt	= 0;

/*@
	assigns systemMillis, systemMilliFractional, systemMonotonicOverflowCnt;
*/
ISR(TIMER0_OVF_vect) {
	unsigned long int m, f;

	m = systemMillis;
	f = systemMilliFractional;

	m = m + SYSCLK_MILLI_INCREMENT;
	f = f + SYSCLK_MILLIFRACT_INCREMENT;

	if(f >= SYSCLK_MILLIFRACT_MAXIMUM) {
		f = f - SYSCLK_MILLIFRACT_MAXIMUM;
		m = m + 1;
	}

	systemMonotonicOverflowCnt = systemMonotonicOverflowCnt + 1;

	systemMillis = m;
	systemMilliFractional = f;
}

/*
	Millis function used to calculate time delays
*/
/*@
	requires \valid(&SREG);
	requires \valid(&systemMillis);

	assigns SREG;
*/
unsigned long int millis() {
	unsigned long int m;

	/*
		Note that this is a hack.
		systemMillis is a multi-byte value so we disable interrupts to read
		consistently BUT this is implementation dependent on the compiler
	*/
	uint8_t srOld = SREG;
	#ifndef FRAMAC_SKIP
		cli();
	#endif
	m = systemMillis;
	SREG = srOld;

	return m;
}

/*@
	requires \valid(&SREG);
	assigns SREG;
*/
unsigned long int micros() {
	uint8_t srOld = SREG;
	unsigned long int overflowCounter;
	unsigned long int timerCounter;

	#ifndef FRAMAC_SKIP
		cli();
	#endif
	overflowCounter = systemMonotonicOverflowCnt;
	timerCounter = TCNT0;

	/*
		Check for pending overflow that has NOT been handeled up to now
	*/
	if(((TIFR0 & 0x01) != 0) && (timerCounter < 255)) {
		overflowCounter = overflowCounter + 1;
	}

	SREG = srOld;

	return ((overflowCounter << 8) + timerCounter) * (64L / (F_CPU / 1000000L));
}

/*@
	requires millisecs >= 0;
	requires \valid(&SREG);
	assigns SREG;
*/
void delay(unsigned long millisecs) {
	//uint16_t lastMicro;
	unsigned int lastMicro;
	/*
		Busy waiting the desired amount of milliseconds ... by
		polling mircos
	*/
	lastMicro = (unsigned int)micros();
	/*@
		loop assigns lastMicro;
		loop assigns millisecs;
	*/
	while(millisecs > 0) {
		// uint16_t curMicro = (uint16_t)micros();
		unsigned int curMicro = micros();
		if(curMicro - lastMicro >= 1000)  {
			/* Every ~ thousand microseconds tick ... */
			lastMicro = lastMicro + 1000;
			millisecs = millisecs - 1;
		}
	}
	return;
}

/*@
	requires microDelay >= 13;

	assigns \nothing;
*/
void delayMicros(unsigned int microDelay) {
	#if F_CPU == 20000000L
		/*
			Burn two additional cycles - together with function
			calling overhead of avr-gcc and the subtraction below
			this should lead to 1 us delay (see assembly output!)
		*/
		__asm__ __volatile__ (
			"nop\n"
			"nop\n"
		);
		if((microDelay = microDelay - 1) == 0) {
			return;
		}

		/*
			Multiply by 5 - loop below takes 4 cycles.
			One cycle takes 1/20000000 seconds i.e. 5e-8 s i.e. 0.05 us.
			One loop iteration burns 4 cycles i.e. 0.2 us
			So we require 5 loop iterations per loop to reach 1 us
		*/
		microDelay = (microDelay << 2) + microDelay;
	#elif F_CPU == 16000000L
		/*
			Function calling, subtraction and conditional
			branch should be equal to approx. 1us (see assembly
			output to fine-tune).
		*/
		if((microDelay = microDelay - 1) == 0) {
			return;
		}

		/*
			Each cycle takes 1/16000000 seconds i.e. 6.25e-8 s i.e. 0.0625 us.
			One loop iteration burns 4 cycles i.e. 0.25 us
			So we require 4 loop iterations to reach 1 us

			This calculation takes us approx. 0.5 us (see assembly
			output to fine tune)
		*/
		microDelay = (microDelay << 2) - 2;
	#elif F_CPU == 8000000L
		if((microDelay = microDelay - 1) == 0) {
			return;
		}
		if((microDelay = microDelay - 1) == 0) {
			return;
		}

		/*
			Each loop iteration burns 0.5us,
			the calculation takes approx 0.5us
			(see assembly output to fine-tune again)
		*/
		microDelay = (microDelay << 1) - 1;
	#else
		#error No known delay loop calibration available for this F_CPU
	#endif

	/*
		Busy waiting loop.
		Takes 4 cycles. Micro Delay has been modified above
	*/
	#ifndef FRAMAC_SKIP
		/*@
			assigns microDelay;
			ensures microDelay == 0;
		*/
		__asm__ __volatile__ (
			"lp: sbiw %0, 1\n"
			"    brne lp"
			: "=w" (microDelay)
			: "0" (microDelay)
		);
	#else
		/*@
			loop assigns microDelay;
			loop invariant 0 <= microDelay;
		*/
		while(microDelay > 0) {
			microDelay = microDelay - 1;
		}
	#endif
	/*@ ghost  */
	return;
}

/*@
	axiomatic hardware_registers {
		axiom valid_PCICR: \valid(&PCICR);
		axiom valid_PCMSK1: \valid(&PCMSK1);

		axiom valid_TCCR0A: \valid(&TCCR0A);
		axiom valid_TCCR0B: \valid(&TCCR0B);
		axiom valid_TCNT0: \valid(&TCNT0);
		axiom valid_TIMSK2: \valid(&TIMSK0);
		axiom valid_OCR0A: \valid(&OCR0A);

		axiom valid_TCCR2A: \valid(&TCCR2A);
		axiom valid_TCCR2B: \valid(&TCCR2B);
		axiom valid_TCNT2: \valid(&TCNT2);
		axiom valid_TIMSK0: \valid(&TIMSK2);
		axiom valid_OCR2A: \valid(&OCR2A);

		axiom valid_PORTB: \valid(&PORTB);	axiom valid_DDRB: \valid(&DDRB);	axiom valid_PINB: \valid(&PINB);
		axiom valid_PORTC: \valid(&PORTC);	axiom valid_DDRC: \valid(&DDRC);	axiom valid_PINC: \valid(&PINC);
		axiom valid_PORTD: \valid(&PORTD);	axiom valid_DDRD: \valid(&DDRD);	axiom valid_PIND: \valid(&PIND);

		axiom valid_UCSR0B: \valid(&UCSR0B);

		axiom valid_TWAR: \valid(&TWAR);	axiom valid_TWCR: \valid(&TWCR);	axiom valid_TWDR: \valid(&TWDR);
	}
*/
/*@
	requires \valid(&TCCR0A) && \valid(&TCCR0B) && \valid(&TIMSK0);
	requires \valid(&PORTB) && \valid(&DDRB);
	requires \valid(&UCSR0B);

	ensures UCSR0B == 0;
*/
int main() {
	#ifndef FRAMAC_SKIP
		cli();
	#endif

	// Setup TIMER0 as our sysclk timer
	// TCCR0A = 0x02;		/* CTC mode */
	TCCR0A = 0x00;
	TCCR0B = 0x03;		/* /64 prescaler */
	TIMSK0 = 0x01;		/* Enable overflow interrupt */

	// Sysclk is now working - re-enable interrupts
	sei();

	#ifdef DEBUG
		// Signal bootup via LED
		DDRB = 0x20;
		PORTB = 0x20;
		delay(1000);
		PORTB = 0x00;
		delay(1000);
		PORTB = 0x20;
		delay(1000);
		PORTB = 0x00;
		delay(1000);
	#endif

	// Disable serial (enabled from bootloader)
	UCSR0B = 0;

	// Initialize I2C
	i2cSlaveInit(STEPPER_I2C_ADDRESS);

	// Stepper setup
	stepperSetup();

	/*
		Since we don't use "delay" in production any more we
		can disable TIMER0 again to reduce possible jitter
		of TIMER2.
	*/
	#ifndef DEBUG
		TIMSK0 = 0x00; /* Disable interrupts of TIMER0 */
	#endif

	for(;;) {
		/*
			Handle stepper events (NOT from ISR but synchronously)
		*/
		handleTimer2Interrupt();
		/*
			Execute I2C message loop
		*/
		i2cMessageLoop();
	}
}
