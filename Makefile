CXXFLAGS=-std=c++14 -Wall
CXXFLAGS+=-g
CXXFLAGS+=-DNOISY
LDLIBS=-lwiringPi

cross_program: cross_program.cc

.PHONY: clean
clean:
	rm cross_program
