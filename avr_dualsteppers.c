#ifndef ARDUINO
	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include <math.h>
#endif
#include <stdint.h>

#ifndef __cplusplus
	typedef int bool;
	#define true 1
	#define false 0
#endif

#ifdef ARDUINO
	#define pinDir1     2
	#define pinStep1    3
	#define pinFault1   16

	#define pinDir2     14
	#define pinStep2    15
	#define pinFault2   17

	/*
	    Shared pins
	      Sleep:    Active low; Pullup to enable
	      Reset:    Active low; Pullup to enable
	      Mode:     0 1 2
	                0 0 0   Full Step
	                1 0 0   1/2
	                0 1 0   1/4
	                1 1 0   1/8
	                0 0 1   1/16
	                1 0 1   1/32
	                0 1 1   1/32
	                1 1 1   1/32
	      Enable:   Active Low; Pull UP to disable
	 */
	#define pinSleep    4
	#define pinReset    5
	#define pinMode2    6
	#define pinMode1    7
	#define pinMode0    8
	#define pinEnable   9
#else
/*
	Pin usage:
		pinDir1		2	PD2		OUT
		pinStep1	3	PD3		OUT

		pinFault1	16	PC2		IN
		pinDir2		14	PC0		OUT
		pinStep2	15	PC1		OUT
		pinFault2	17	PC3		IN

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
#endif

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

#ifndef ARDUINO
	/*
		We use our own systick implementation
	*/
	unsigned long int millis();
	unsigned long int micros();
	void delay(unsigned long millisecs);
	void delayMicros(unsigned int microDelay);
#endif


enum stepperCommandType {
	stepperCommand_AccelerateStopToStop,
	stepperCommand_ConstantSpeed,
	stepperCommand_Stop,
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
	double					c_i;				/* The previously loaded c_i (used during calculation and advancing) */
	uint32_t				counterCurrent;		/* Counter reduced every time until we reach zero */

	struct stepperCommand	cmdQueue[STEPPER_COMMANDQUEUELENGTH];
	unsigned int			cmdQueueHead;
	unsigned int			cmdQueueTail;

	#ifdef ARDUINO
		int						pinStep;
		int						pinDir;
		int						pinFault;
	#endif

	/* Cache for precalculated constants - see documentation */
	struct {
		double				c1;
		double				c2;
		double				c3;
		double				c4;
		double				c5;
		double				c6;

		double				c7;
		double				c8;
		double				c9;
		double				c10;
	} constants;

	struct {
		double				acceleration; /* in rad/sec */
		double				deceleration; /* in rad/sec */
		double				alpha; /* step size in rad */
		double				vmax; /* in rad/sec */
	} settings;
};



#define STEPPER_COUNT 2

/*
	Declare state for both steppers
*/
volatile static struct stepperState			state[STEPPER_COUNT];


bool bResetRun = false;

ISR(TIMER2_COMPA_vect) {
	/*
		Interrupt handler executed with 2 * STEPPER_TIMERTICK_FRQ frequency
		either use to disable step pins or advance state machine depending on
		out state. We have to finish in way less then 1024 ticks ...
	*/
	for(int stepperIdx = 0; stepperIdx < STEPPER_COUNT; stepperIdx = stepperIdx + 1) {
		if(bResetRun) {
			#ifndef ARDUINO
				if(stepperIdx == 0) {
					// Pulse PD2 low
					PORTD = PORTD & (~0x08);
				} else {
					// Pulse PC1 low
					PORTC = PORTC & (~0x02);
				}
			#else
				digitalWrite(state[stepperIdx].pinStep, LOW);
			#endif
		} else {
			if(state[stepperIdx].cmdQueueTail == state[stepperIdx].cmdQueueHead) {
				/* Nothing to do ... */
				continue;
			}

			const int qIdx = state[stepperIdx].cmdQueueTail;
			if(state[stepperIdx].c_i == -1) {
				/* We wake up from idle ... any may have to do some initialization */
				#ifdef ARDUINO
					digitalWrite(state[stepperIdx].pinDir, (state[stepperIdx].cmdQueue[qIdx].forward != 0) ? HIGH : LOW);
				#else
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
				#endif

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
				} else if(state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nC > 0) {
					state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nC = state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nC - 1;
				} else if(state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nD > 0) {
					state[stepperIdx].c_i = state[stepperIdx].c_i / (1 + state[stepperIdx].constants.c9 * state[stepperIdx].c_i * state[stepperIdx].c_i);
					if((state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nD = state[stepperIdx].cmdQueue[qIdx].data.acceleratedStopToStop.nD - 1) == 0) {
						/* Switch to next state at next interrupt */
						state[stepperIdx].cmdQueueTail = (state[stepperIdx].cmdQueueTail + 1) % STEPPER_COMMANDQUEUELENGTH;
						state[stepperIdx].c_i = -1; /* This will initialize the next state as soon as it is available */
					}
				}
				/* c_i to counter value */
				if(state[stepperIdx].c_i > 4294967295.0) {
					state[stepperIdx].counterCurrent = ~0;
				} else if(state[stepperIdx].c_i < 1.0) {
					state[stepperIdx].counterCurrent = 1;
				} else {
					state[stepperIdx].counterCurrent = (uint32_t)state[stepperIdx].c_i;
				}
			} else if(state[stepperIdx].cmdQueue[qIdx].cmdType == stepperCommand_ConstantSpeed) {
				state[stepperIdx].counterCurrent = state[stepperIdx].c_i; // Counter reload ...
			}

			#ifdef ARDUINO
				digitalWrite(state[stepperIdx].pinStep, HIGH);
			#else
			if(stepperIdx == 0) {
				// Pulse PD2 high
				PORTD = PORTD | 0x08;
			} else {
				// Pulse PC1 high
				PORTC = PORTC | 0x02;
			}
			#endif
		}
	}

	bResetRun = !bResetRun;
}

static bool updateConstants(int stepperIndex) {
	/*
		Expensive update of constants for motion planner
	*/
	if(stepperIndex > STEPPER_COUNT) {
		return false;
	}

	state[stepperIndex].constants.c1 = 1.0 / (2.0 * state[stepperIndex].settings.acceleration * state[stepperIndex].settings.alpha);
	state[stepperIndex].constants.c2 = state[stepperIndex].settings.vmax * state[stepperIndex].settings.vmax * state[stepperIndex].constants.c1;
	state[stepperIndex].constants.c3 = 1.0 / (2.0 * state[stepperIndex].settings.deceleration * state[stepperIndex].settings.alpha);
	state[stepperIndex].constants.c4 = -1.0 * state[stepperIndex].settings.vmax * state[stepperIndex].settings.vmax * state[stepperIndex].constants.c3;
	state[stepperIndex].constants.c5 = state[stepperIndex].settings.deceleration / (state[stepperIndex].settings.deceleration - state[stepperIndex].settings.acceleration);
	state[stepperIndex].constants.c6 = 1.0 / (2.0 * state[stepperIndex].settings.alpha * (state[stepperIndex].settings.deceleration - state[stepperIndex].settings.acceleration));
	state[stepperIndex].constants.c7 = 2 * state[stepperIndex].settings.alpha / state[stepperIndex].settings.acceleration;
	state[stepperIndex].constants.c8 = state[stepperIndex].settings.acceleration / (state[stepperIndex].settings.alpha * (double)STEPPER_TIMERTICK_FRQ * (double)STEPPER_TIMERTICK_FRQ);
	state[stepperIndex].constants.c9 = state[stepperIndex].settings.deceleration / (state[stepperIndex].settings.alpha * (double)STEPPER_TIMERTICK_FRQ * (double)STEPPER_TIMERTICK_FRQ);
	state[stepperIndex].constants.c10 = state[stepperIndex].settings.alpha * (double)STEPPER_TIMERTICK_FRQ;

	return true;
}

static void stepperSetup() {
	/*
		Stop our timer if it's currently running
	*/
	cli();
	TCCR2B = 0; 							/* Disable timer if it's currently running */
	sei();

	/*
		Initialize pins and initial 1/4 microstepping
	*/
	#ifdef ARDUINO
		state[0].pinDir = pinDir1;
		state[0].pinStep = pinStep1;
		state[0].pinFault = pinFault1;

		state[1].pinDir = pinDir2;
		state[1].pinStep = pinStep2;
		state[1].pinFault = pinFault2;
	#endif

	#ifdef ARDUINO
		pinMode(pinDir1, OUTPUT);   digitalWrite(pinDir1, 0);
		pinMode(pinStep1, OUTPUT);  digitalWrite(pinStep1, 0);
		pinMode(pinDir2, OUTPUT);   digitalWrite(pinDir2, 0);
		pinMode(pinStep2, OUTPUT);  digitalWrite(pinStep2, 0);

		pinMode(pinFault1, INPUT);
		pinMode(pinFault2, INPUT);

		pinMode(pinSleep, OUTPUT);  digitalWrite(pinSleep, 0); // Enter sleep mode
		pinMode(pinReset, OUTPUT);  digitalWrite(pinReset, 0); // Enter RESET mode
		pinMode(pinEnable, OUTPUT); digitalWrite(pinEnable, 1); // DISABLE device inputs
		pinMode(pinMode2, OUTPUT);  digitalWrite(pinMode0, 0);
		pinMode(pinMode1, OUTPUT);  digitalWrite(pinMode1, 0);
		pinMode(pinMode0, OUTPUT);  digitalWrite(pinMode2, 0); // Default: Full step mode

		delay(10);

		digitalWrite(pinSleep, 1);  // Leave sleep state
		delay(2);
		digitalWrite(pinEnable, 0); // Enable inputs
		delay(150);
		delay(5);
		// Now we reset the devices
		digitalWrite(pinReset, 0);  // Enter reset mode
		delay(150);
		digitalWrite(pinReset, 1);  // Leave reset mode
		delay(150);
	#else
		DDRB = 0x03;
		PORTB = 0x02;
		DDRC = 0x03;
		PORTC = 0x00;
		DDRD = 0xFC;
		PORTD = 0x00;

		delay(10);
		PORTD = PORTD | 0x10; /* Leave sleep state */
		delay(2);
		PORTB = PORTB & (~0x02); /* Enable inputs */
		delay(150);
		PORTD = PORTD | 0x20; /* Leave reset mode */
	#endif

	/*
		Initialize stepper state machine to zero
	*/
	for(int i = 0; i < STEPPER_COUNT; i=i+1) {
		state[i].cmdQueueHead = 0;
		state[i].cmdQueueTail = 0;
		state[i].counterCurrent = 0;
		state[i].c_i = -1; /* Will trigger initialization after planning */

		state[i].settings.acceleration = STEPPER_INITIAL_ACCELERATION;
		state[i].settings.deceleration = STEPPER_INITIAL_DECELERATION;
		state[i].settings.alpha = STEPPER_INITIAL_ALPHA;
		state[i].settings.vmax = STEPPER_INITIAL_VMAX;

		updateConstants(i);
	}

	/*
		Initialize our timer. We use Timer2 for our purposes
	*/
	TCNT2 = 0; 								/* Set current timer counter to zero */
	TCCR2A = 0x02;							/* CTC Mode (count up to OCR2A), disable OCR output pins */
	OCR2A = 0x01;							/* We count up to one - so trigger every pulse */
	TIMSK2 = 0x02;							/* Set OCIE2A flag to enable interrupts on output compare */
	TCCR2B = STEPPER_TIMERTICK_PRESCALER;	/* Select our prescaler, non FOCA, enable timer */
}

static void stepperPlanMovement_ConstantSpeed(int stepperIndex, double v, int direction) {
	/*
		Determine at which index we want to plan
	*/
	const int idx = state[stepperIndex].cmdQueueHead;

	state[stepperIndex].cmdQueue[idx].cmdType = stepperCommand_ConstantSpeed;

	double tickCount = state[stepperIndex].constants.c10 / v;
	if(tickCount > 4294967295.0) {
		/* Clamp to maximum */
		state[stepperIndex].cmdQueue[idx].data.constantSpeed.cConst = ~0;
	} else if(tickCount < 1.0) {
		/* Clamp to minimum */
		state[stepperIndex].cmdQueue[idx].data.constantSpeed.cConst = 1;
	} else {
		state[stepperIndex].cmdQueue[idx].data.constantSpeed.cConst = (uint32_t)(state[stepperIndex].constants.c10 / v);
	}

	state[stepperIndex].cmdQueue[idx].forward = direction;

	/*
		Serial.print("Constant speed movement planned: ");
		Serial.println(state[stepperIndex].cmdQueue[idx].data.constantSpeed.cConst);
	*/

	/*
		Make command active
	*/
	state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
	return;
}

static void stepperPlanMovement_AccelerateStopToStop(int stepperIndex, double sTotal, int direction) {
	const int idx = state[stepperIndex].cmdQueueHead;

	double nTotal = sTotal / state[stepperIndex].settings.alpha;

	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA = state[stepperIndex].constants.c2;
	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD = state[stepperIndex].constants.c4;

	uint32_t nTP = state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA + state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD;
	if(nTP < nTotal) {
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nC = nTotal - nTP;
	} else {
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA = nTotal * state[stepperIndex].constants.c5;
		state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD = nTotal - state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA;
	}
	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.c7 = state[stepperIndex].constants.c7;
	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.c8 = state[stepperIndex].constants.c8;
	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.c9 = state[stepperIndex].constants.c9;

	state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.initialDelayTicks = sqrt(state[stepperIndex].constants.c7) * (double)STEPPER_TIMERTICK_FRQ;

	state[stepperIndex].cmdQueue[idx].forward = direction;

	/*
		Serial.print("Planned accelerated move to, acceleration steps ");
		Serial.print(state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nA);
		Serial.print(", deceleration steps ");
		Serial.print(state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nC);
		Serial.print(", constant steps ");
		Serial.println(state[stepperIndex].cmdQueue[idx].data.acceleratedStopToStop.nD);
		delay(1000);
	*/

	state[stepperIndex].cmdQueueHead = (state[stepperIndex].cmdQueueHead + 1) % STEPPER_COMMANDQUEUELENGTH;
	return;
}


#ifndef ARDUINO
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
	unsigned long int millis() {
		unsigned long int m;

		/*
			Note that this is a hack.
			systemMillis is a multi-byte value so we disable interrupts to read
			consistently BUT this is implementation dependent on the compiler
		*/
		uint8_t srOld = SREG;
		cli();
		m = systemMillis;
		SREG = srOld;

		return m;
	}

	unsigned long int micros() {
		uint8_t srOld = SREG;
		unsigned long int overflowCounter;
		unsigned long int timerCounter;

		cli();
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

	void delay(unsigned long millisecs) {
		uint16_t lastMicro;
		/*
			Busy waiting the desired amount of milliseconds ... by
			polling mircos
		*/
		lastMicro = (uint16_t)micros();
		while(millisecs > 0) {
			if(((uint16_t)micros() - lastMicro) >= 1000) {
				/* Every ~ thousand microseconds tick ... */
				lastMicro = lastMicro + 1000;
				millisecs = millisecs - 1;
			}
		}
		return;
	}
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
		__asm__ __volatile__ (
			"lp: sbiw %0, 1\n"
			"    brne lp"
			: "=w" (microDelay)
			: "0" (microDelay)
		);
		return;
	}


	int main() {
		cli();

		// Setup TIMER0 as our sysclk timer
		// TCCR0A = 0x02;		/* CTC mode */
		TCCR0A = 0x00;
		TCCR0B = 0x03;		/* /64 prescaler */
		TIMSK0 = 0x01;		/* Enable overflow interrupt */

		// Sysclk is now working - re-enable interrupts
		sei();

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

		// Setup serial
		#ifdef STEPPER_ENABLE_UART
			/* ToDo */
		#else
			UCSR0B = 0;
		#endif

		// Stepper setup
		stepperSetup();
		// Some test code
		// stepperPlanMovement_ConstantSpeed(0, 6.28318, 0);
		// stepperPlanMovement_ConstantSpeed(1, 6.28318/2, 1);

		bool fwd = true;
		for(;;) {
			// Just some Test Code every 10 seconds
			delay(10000);
			stepperPlanMovement_AccelerateStopToStop(0, 62.83185307179586476925286766559, fwd ? 1 : 0);
			stepperPlanMovement_AccelerateStopToStop(1, 62.83185307179586476925286766559/2, fwd ? 1 : 0);
			fwd = !fwd;
			// stepperPlanMovement_AccelerateStopToStop(0, 62.83185307179586476925286766559);
		}
	}
#else
	void setup() {
		Serial.begin(115200);
		while(!Serial) { }
		stepperSetup();
		// Some test code
		// stepperPlanMovement_ConstantSpeed(0, 6.28318, 0);
		// stepperPlanMovement_ConstantSpeed(1, 6.28318/2, 1);

	}

	bool fwd = true;
	void loop() {
		// Just some Test Code
		delay(10000);
		stepperPlanMovement_AccelerateStopToStop(0, 62.83185307179586476925286766559, fwd ? 1 : 0);
		stepperPlanMovement_AccelerateStopToStop(1, 62.83185307179586476925286766559/2, fwd ? 1 : 0);
		fwd = !fwd;
		// stepperPlanMovement_AccelerateStopToStop(0, 62.83185307179586476925286766559);
	}
#endif
