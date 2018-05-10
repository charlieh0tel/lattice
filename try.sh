#!/bin/sh

set -o xtrace
set -o errexit

make

# if compile with Wiring
#
# Wiring calls pin 37 GPIO 25.
# gpio unexportall
#./cross_program  /dev/i2c-1  0x40 25 ./gpio_toggle_impl1_original.bit

# Kernel calls pin 37 GPIO 26.
gpio export 26 output
./cross_program  /dev/i2c-1  0x40 26 ./gpio_toggle_impl1_original.bit
