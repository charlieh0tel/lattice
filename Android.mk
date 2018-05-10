LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE := cross_program
LOCAL_SRC_FILES := \
	cross_program.cc io.cc
LOCAL_LDLIBS :=
LOCAL_STATIC_LIBRARIES :=
LOCAL_SHARED_LIBRARIES :=
include $(BUILD_EXECUTABLE)

