CXXFLAGS=-std=c++14 -Wall
CXXFLAGS+=-g
CXXFLAGS+=-DNOISY
#CXXFLAGS+=-DWIRING_PI
#LDLIBS=-lwiringPi

cross_program: cross_program.cc io.cc

cross_program.cc io.cc: io.h

.PHONY: clean
clean:
	rm cross_program
