
ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:=               \
    AudioPolicyManager.cpp

LOCAL_SHARED_LIBRARIES := \
    libcutils \
    libutils \
    libmedia

LOCAL_STATIC_LIBRARIES := libaudiopolicybase

LOCAL_MODULE:= libaudiopolicy

ifeq ($(BOARD_HAVE_BLUETOOTH),true)
  LOCAL_CFLAGS += -DWITH_A2DP
endif

include $(BUILD_SHARED_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := libaudio

LOCAL_SHARED_LIBRARIES := \
    libcutils \
    libutils \
    libmedia \
    libhardware_legacy

ifeq ($TARGET_OS)-$(TARGET_SIMULATOR),linux-true)
LOCAL_LDLIBS += -ldl
endif

ifneq ($(TARGET_SIMULATOR),true)
LOCAL_SHARED_LIBRARIES += libdl
endif

LOCAL_SRC_FILES += AudioHardware.cpp

LOCAL_CFLAGS += -fno-short-enums

LOCAL_STATIC_LIBRARIES += libaudiointerface
ifeq ($(BOARD_HAVE_BLUETOOTH),true)
  LOCAL_SHARED_LIBRARIES += liba2dp
endif

ifeq ($(BOARD_HAVE_FM_RADIO),true)
ifeq ($(BOARD_WLAN_DEVICE),bcm4329)
	LOCAL_CFLAGS += -DHAVE_BCM_FM_RADIO
endif
ifeq ($(BOARD_WLAN_DEVICE),wl1251)
	LOCAL_CFLAGS += -DHAVE_TI_FM_RADIO
	LOCAL_CFLAGS += -DWL1251
endif
ifeq ($(BOARD_WLAN_DEVICE),wl1271)
	LOCAL_CFLAGS += -DHAVE_TI_FM_RADIO
endif
endif

include $(BUILD_SHARED_LIBRARY)

endif # not BUILD_TINY_ANDROID

