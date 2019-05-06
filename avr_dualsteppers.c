#include <stdint.h>

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
#define STEPPER_INITIAL_ACCELERATION	6
#define STEPPER_INITIAL_DECELERATION	-6


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

	int						pinStep;
	int						pinDir;
	int						pinFault;

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
			digitalWrite(state[stepperIdx].pinStep, LOW);
		} else {
			if(state[stepperIdx].cmdQueueTail == state[stepperIdx].cmdQueueHead) {
				/* Nothing to do ... */
				continue;
			}

			const int qIdx = state[stepperIdx].cmdQueueTail;
			if(state[stepperIdx].c_i == -1) {
				/* We wake up from idle ... any may have to do some initialization */
				digitalWrite(state[stepperIdx].pinDir, (state[stepperIdx].cmdQueue[qIdx].forward != 0) ? HIGH : LOW);

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

			digitalWrite(state[stepperIdx].pinStep, HIGH);
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
	state[0].pinDir = pinDir1;
	state[0].pinStep = pinStep1;
	state[0].pinFault = pinFault1;
	
	state[1].pinDir = pinDir2;
	state[1].pinStep = pinStep2;
	state[1].pinFault = pinFault2;

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
