all: 001_accelerations 002_disabledrivers

001_accelerations: 001_accelerations.c ../libi2cstepper.a

	clang -Wall -ansi -std=c99 -pedantic -o 001_accelerations 001_accelerations.c -L../ -li2cstepper

002_disabledrivers: 002_disabledrivers.c ../libi2cstepper.a

		clang -Wall -ansi -std=c99 -pedantic -o 002_disabledrivers 002_disabledrivers.c -L../ -li2cstepper
