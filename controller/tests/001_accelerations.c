#include <stdio.h>
#include <stdlib.h>
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
	i2cClose(bus2c); bus2c = NULL;
	printf("Done\n");
	return 0;
}
