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


	printf("Running accelerated movement with default settings\n");
	step->obj.vtbl->setMicrosteps(step, 0, 0); // Setting on a single channel sets for both channels
	step->obj.vtbl->queueMoveAngularDistance(step, 1, -62.83185307179586476925286766559/5.0);
	step->obj.vtbl->queueMoveAngularDistance(step, 0, 62.83185307179586476925286766559/5.0);
	sleep(3);
	step->obj.vtbl->queueMoveAngularDistance(step, 1, 62.83185307179586476925286766559/5.0);
	step->obj.vtbl->queueMoveAngularDistance(step, 0, -62.83185307179586476925286766559/5.0);
	sleep(10);
	step->obj.vtbl->queueConstantSpeed(step, 0, 6);
	step->obj.vtbl->queueConstantSpeed(step, 1, 3);
	sleep(10);
	step->obj.vtbl->execMoveAngularDistance(step, 1, -62.83185307179586476925286766559/5.0);
	step->obj.vtbl->execMoveAngularDistance(step, 0, 62.83185307179586476925286766559/5.0);
	sleep(20);

	printf("Running accelerated movement with different microsteps (1/4)\n");
	step->obj.vtbl->setMicrosteps(step, 0, 2); // Setting on a single channel sets for both channels

	step->obj.vtbl->queueMoveAngularDistance(step, 1, -62.83185307179586476925286766559/5.0);
	step->obj.vtbl->queueMoveAngularDistance(step, 0, 62.83185307179586476925286766559/5.0);
	sleep(3);
	step->obj.vtbl->queueMoveAngularDistance(step, 1, 62.83185307179586476925286766559/5.0);
	step->obj.vtbl->queueMoveAngularDistance(step, 0, -62.83185307179586476925286766559/5.0);
	sleep(10);
	step->obj.vtbl->queueConstantSpeed(step, 0, 6);
	step->obj.vtbl->queueConstantSpeed(step, 1, 3);
	sleep(10);
	step->obj.vtbl->execMoveAngularDistance(step, 1, -62.83185307179586476925286766559/5.0);
	step->obj.vtbl->execMoveAngularDistance(step, 0, 62.83185307179586476925286766559/5.0);
	sleep(20);

	printf("Running accelerated movement with different microsteps (1/32)\n");
	step->obj.vtbl->setMicrosteps(step, 0, 32); // Setting on a single channel sets for both channels

	step->obj.vtbl->queueMoveAngularDistance(step, 1, -62.83185307179586476925286766559/5.0);
	step->obj.vtbl->queueMoveAngularDistance(step, 0, 62.83185307179586476925286766559/5.0);
	sleep(3);
	step->obj.vtbl->queueMoveAngularDistance(step, 1, 62.83185307179586476925286766559/5.0);
	step->obj.vtbl->queueMoveAngularDistance(step, 0, -62.83185307179586476925286766559/5.0);
	sleep(10);
	step->obj.vtbl->queueConstantSpeed(step, 0, 6);
	step->obj.vtbl->queueConstantSpeed(step, 1, 3);
	sleep(10);
	step->obj.vtbl->execMoveAngularDistance(step, 1, -62.83185307179586476925286766559/5.0);
	step->obj.vtbl->execMoveAngularDistance(step, 0, 62.83185307179586476925286766559/5.0);
	sleep(20);



	printf("Waiting to disable drivers in 60 seconds\n");
	sleep(60);
	step->obj.vtbl->execDisableDriver(step, 0);
	step->obj.vtbl->execDisableDriver(step, 1);

	sleep(1);
	printf("Releasing\n");
	step->obj.vtbl->release(step);


	i2cClose(bus2c); bus2c = NULL;
	printf("Done\n");
	return 0;
}
