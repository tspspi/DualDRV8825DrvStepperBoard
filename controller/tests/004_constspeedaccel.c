#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include "../dualstep.h"

int main(int argc, char* argv[]) {
	struct busI2C* bus2c;
	enum i2cError e2c;

	struct i2cDualStepper* step;
	enum i2cDualStepperError estep;

	e2c = i2cOpen(&bus2c, NULL);
	if(e2c != i2cE_Ok) {
		printf("Failed to open bus: %u\n", e2c);
		return -1;
	}

	estep = i2cDualStepper_Open(&step, 0x14, bus2c);
	if(estep != i2cDualStepperErrorE_Ok) {
		printf("Failed to open stepper: %u\n", estep);
		i2cClose(bus2c); bus2c = NULL;
		return -1;
	}
	printf("Stepper opened\n");

	printf("Querying acceleration and deceleration: ");
	union {
		double accel;
		uint64_t raw;
	} a;
	union {
		double decel;
		uint64_t raw;
	} d;
	step->obj.vtbl->getAccelerationDeceleration(step, 0, &a.accel, &d.decel);
	printf("%lf and %lf (raw %08llx %08llx)\n", a.accel, d.decel, a.raw, d.raw);


	printf("Using full steps (default setting)\n");
	step->obj.vtbl->setMicrosteps(step, 0, 0); // Setting on a single channel sets for both channels

	printf("Accelerating to half maximum speed\n");
	step->obj.vtbl->queueAccelerateToSpeed(step, 0, 3);
	step->obj.vtbl->queueConstantSpeed(step, 0, 3);

	step->obj.vtbl->queueAccelerateToSpeed(step, 1, 3);
	step->obj.vtbl->queueConstantSpeed(step, 1, 3);

	printf("Sleeping 5 seconds, then accelerating to max speed\n");
	sleep(5);
	step->obj.vtbl->execAccelerateToSpeed(step, 0, 6);
	step->obj.vtbl->queueConstantSpeed(step, 0, 6);

	step->obj.vtbl->execAccelerateToSpeed(step, 1, 6);
	step->obj.vtbl->queueConstantSpeed(step, 1, 6);

	printf("Sleeping 5 seconds, then decelerating to half\n");
	sleep(5);
	step->obj.vtbl->execAccelerateToSpeed(step, 0, 3);
	step->obj.vtbl->queueConstantSpeed(step, 0, 3);

	step->obj.vtbl->execAccelerateToSpeed(step, 1, 3);
	step->obj.vtbl->queueConstantSpeed(step, 1, 3);

	printf("Sleeping 5 seconds, then decelerating to half in other direction\n");
	sleep(5);
	step->obj.vtbl->execAccelerateToSpeed(step, 0, -3);
	step->obj.vtbl->queueConstantSpeed(step, 0, -3);

	step->obj.vtbl->execAccelerateToSpeed(step, 1, -3);
	step->obj.vtbl->queueConstantSpeed(step, 1, -3);

	printf("Sleeping 5 seconds, then decelerating to zero speed followed by disabling stepper\n");
	sleep(5);
	step->obj.vtbl->execAccelerateToSpeed(step, 0, 0);
	step->obj.vtbl->execAccelerateToSpeed(step, 1, 0);

	step->obj.vtbl->queueDisableDriver(step, 0);
	step->obj.vtbl->queueDisableDriver(step, 1);

	printf("Waiting 5 seconds\n");
	sleep(5);

	printf("Releasing\n");
	step->obj.vtbl->release(step);


	i2cClose(bus2c); bus2c = NULL;
	printf("Done\n");
	return 0;
}
