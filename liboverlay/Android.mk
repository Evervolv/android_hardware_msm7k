# Copyright (C) 2008 The Android Open Source Project
# Copyright (c) 2009, Code Aurora Forum. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License
#

LOCAL_PATH := $(call my-dir)

# HAL module implemenation, not prelinked and stored in
# hw/<OVERLAY_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SHARED_LIBRARIES := liblog
LOCAL_SRC_FILES := overlay.cpp
LOCAL_MODULE := overlay.default
LOCAL_C_INCLUDES += hardware/msm7k/libgralloc-qsd8k
LOCAL_CFLAGS := -DCONFIG_MSM_MDP40
LOCAL_MODULE_TAGS := optional
include $(BUILD_SHARED_LIBRARY)
