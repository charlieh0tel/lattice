#!/bin/sh

set -o xtrace
set -o errexit

make

# if compiled with Wiring
# Wiring calls pin 37 GPIO 25.
#./cross_program  /dev/i2c-1  0x40 25 ./gpio_toggle_impl1_original.bit

# if not compield with Wiring
# Kernel calls pin 37 GPIO 26.
./cross_program  /dev/i2c-1  0x40 26 ./gpio_toggle_impl1_original.bit
