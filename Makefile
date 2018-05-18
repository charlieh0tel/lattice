CXXFLAGS=-std=c++14 -Wall
CXXFLAGS+=-g
#CXXFLAGS+=-DNOISY
CXXFLAGS+=-DNEED_I2C_H
#CXXFLAGS+=-DWIRING_PI
#LDLIBS=-lwiringPi

read_i2c_hall: read_i2c_hall.cc io.cc

read_i2c_hall.cc io.cc: io.h

.PHONY: clean
clean:
	rm read_i2c_hall
