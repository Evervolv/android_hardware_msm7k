# When zero we link against libqcamera; when 1, we dlopen libqcamera.
DLOPEN_LIBQCAMERA:=1

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_CFLAGS:=-fno-short-enums
LOCAL_CFLAGS+=-DDLOPEN_LIBQCAMERA=$(DLOPEN_LIBQCAMERA)

LOCAL_SRC_FILES:= QualcommCameraHardware.cpp

LOCAL_SHARED_LIBRARIES:= libutils libui liblog
ifneq ($(DLOPEN_LIBQCAMERA),1)
LOCAL_SHARED_LIBRARIES+= libqcamera
else
LOCAL_SHARED_LIBRARIES+= libdl
endif

LOCAL_MODULE:= libcamera

include $(BUILD_SHARED_LIBRARY)
