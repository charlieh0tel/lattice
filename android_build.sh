#!/bin/sh

set -o errexit

PATH=$HOME/android/android-ndk-r17:$PATH

export NDK_PROJECT_PATH=.
ndk-build NDK_APPLICATION_MK=./Application.mk
