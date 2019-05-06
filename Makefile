CPUFREQ=16000000L
FLASHDEV=/dev/ttyU0

all: avr_dualsteppers.hex

avr_dualsteppers.bin: avr_dualsteppers.c

	avr-gcc -Wall -Os -mmcu=atmega328p -DF_CPU=$(CPUFREQ) -o avr_dualsteppers.bin avr_dualsteppers.c

avr_dualsteppers.hex: avr_dualsteppers.bin

	avr-size -t avr_dualsteppers.bin
	avr-objcopy -j .text -j .data -O ihex avr_dualsteppers.bin avr_dualsteppers.hex

flash: avr_dualsteppers.hex

	sudo chmod 666 $(FLASHDEV)
	avrdude -v -p atmega328p -c arduino -P /dev/ttyU0 -b 57600 -D -U flash:w:avr_dualsteppers.hex:i

clean:

	-rm *.bin

cleanall: clean

	-rm *.hex

.PHONY: all clean cleanall
