all: 001_accelerations 002_disabledrivers 003_microstepping 004_constspeedaccel

001_accelerations: 001_accelerations.c ../libi2cstepper.a

	clang -Wall -ansi -std=c99 -pedantic -o 001_accelerations 001_accelerations.c -L../ -li2cstepper

002_disabledrivers: 002_disabledrivers.c ../libi2cstepper.a

	clang -Wall -ansi -std=c99 -pedantic -o 002_disabledrivers 002_disabledrivers.c -L../ -li2cstepper

003_microstepping: 003_microstepping.c ../libi2cstepper.a

	clang -Wall -ansi -std=c99 -pedantic -o 003_microstepping 003_microstepping.c -L../ -li2cstepper

004_constspeedaccel: 004_constspeedaccel.c ../libi2cstepper.a

	clang -Wall -ansi -std=c99 -pedantic -o 004_constspeedaccel 004_constspeedaccel.c -L../ -li2cstepper
