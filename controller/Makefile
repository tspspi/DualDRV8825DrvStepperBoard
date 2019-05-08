CC=clang -Wall -ansi -std=c99 -pedantic
CCCOMPILE=$(CC) -c
CCLINK=$(CC)

all: libi2cstepper.a clean

clean:

	-rm *.o

cleanall: clean

	-rm *.a

dualstep.o: dualstep.c dualstep.h i2c.o

	$(CCCOMPILE) -o dualstep.o dualstep.c

i2c.o: i2c.c i2c.h

	$(CCCOMPILE) -o i2c.o i2c.c

libi2cstepper.a: i2c.o dualstep.o

	ar rcs libi2cstepper.a i2c.o dualstep.o

.PHONY: all clean cleanall
