CPUFREQ=16000000L
FLASHDEV=/dev/ttyU0
I2CADR=0x14

all: avr_dualsteppers.hex

avr_dualsteppers.bin: avr_dualsteppers.c

	avr-gcc -Wall -Os -mmcu=atmega328p -DF_CPU=$(CPUFREQ) -DSTEPPER_I2C_ADDRESS=$(I2CADR) -o avr_dualsteppers.bin avr_dualsteppers.c

avr_dualsteppers.hex: avr_dualsteppers.bin

	avr-size -t avr_dualsteppers.bin
	avr-objcopy -j .text -j .data -O ihex avr_dualsteppers.bin avr_dualsteppers.hex

flash: avr_dualsteppers.hex

	sudo chmod 666 $(FLASHDEV)
	avrdude -v -p atmega328p -c arduino -P /dev/ttyU0 -b 57600 -D -U flash:w:avr_dualsteppers.hex:i

framac: avr_dualsteppers.c

	-rm framacreport.csv
	frama-c -wp-verbose 0 -wp -rte -wp-rte -wp-dynamic -wp-timeout 300 -cpp-extra-args="-I/usr/home/tsp/framaclib/ -DF_CPU=16000000L -D__AVR_ATmega328P__ -DFRAMAC_SKIP" avr_dualsteppers.c -then -no-unicode -report -report-csv framacreport.csv

clean:

	-rm *.bin

cleanall: clean

	-rm *.hex

.PHONY: all clean cleanall
