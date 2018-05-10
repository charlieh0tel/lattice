#!/bin/sh

set -o errexit

PATH=$HOME/android/android-ndk-r14b:$PATH

export NDK_PROJECT_PATH=.
ndk-build NDK_APPLICATION_MK=./Application.mk
