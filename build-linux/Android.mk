#
# Copyright (C) YuqiaoZhang(HanetakaChou)
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

# https://developer.android.com/ndk/guides/android_mk

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := mediapipe

LOCAL_SRC_FILES := $(LOCAL_PATH)/../thirdparty/mediapipe/build-linux/bin/$(TARGET_ARCH_ABI)/libmediapipe$(TARGET_SONAME_EXTENSION)

include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)

LOCAL_MODULE := OpenCL

LOCAL_SRC_FILES := $(LOCAL_PATH)/../thirdparty/mediapipe/build-linux/bin/$(TARGET_ARCH_ABI)/libOpenCL$(TARGET_SONAME_EXTENSION)

include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)

LOCAL_MODULE := BRX-Motion-MediaPipe-Model-Asset

LOCAL_SRC_FILES := \
    $(LOCAL_PATH)/../models/hand_landmarker_task.cpp \
    $(LOCAL_PATH)/../models/face_landmarker_task.cpp \
    $(LOCAL_PATH)/../models/pose_landmarker_task.cpp

LOCAL_CFLAGS :=

ifeq (armeabi-v7a,$(TARGET_ARCH_ABI))
LOCAL_ARM_MODE := arm
LOCAL_ARM_NEON := true
else ifeq (arm64-v8a,$(TARGET_ARCH_ABI))
LOCAL_CFLAGS +=
else ifeq (x86,$(TARGET_ARCH_ABI))
LOCAL_CFLAGS += -mf16c
LOCAL_CFLAGS += -mfma
LOCAL_CFLAGS += -mavx2
else ifeq (x86_64,$(TARGET_ARCH_ABI))
LOCAL_CFLAGS += -mf16c
LOCAL_CFLAGS += -mfma
LOCAL_CFLAGS += -mavx2
else
LOCAL_CFLAGS +=
endif

LOCAL_CFLAGS += -Wall
LOCAL_CFLAGS += -Werror=return-type

LOCAL_C_INCLUDES :=

LOCAL_CPPFLAGS := 
LOCAL_CPPFLAGS += -std=c++20

LOCAL_LDFLAGS :=
LOCAL_LDFLAGS += -Wl,--enable-new-dtags
LOCAL_LDFLAGS += -Wl,-rpath,\$$ORIGIN
LOCAL_LDFLAGS += -Wl,--version-script,$(LOCAL_PATH)/BRX-Motion-MediaPipe-Model-Asset.map

LOCAL_STATIC_LIBRARIES :=

LOCAL_SHARED_LIBRARIES :=

include $(BUILD_SHARED_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := BRX-Motion

LOCAL_SRC_FILES := \
	$(LOCAL_PATH)/../source/brx_animation_ik_ccd.cpp \
	$(LOCAL_PATH)/../source/brx_animation_ik_one_joint.cpp \
	$(LOCAL_PATH)/../source/brx_animation_ik_two_joints.cpp \
	$(LOCAL_PATH)/../source/brx_motion_animation.cpp \
	$(LOCAL_PATH)/../source/brx_motion_media_pipe_video_detector.cpp \
	$(LOCAL_PATH)/../source/brx_motion_opencv_video_capture.cpp \
	$(LOCAL_PATH)/../source/internal_tflite.cpp

LOCAL_CFLAGS :=

ifeq (armeabi-v7a,$(TARGET_ARCH_ABI))
LOCAL_ARM_MODE := arm
LOCAL_ARM_NEON := true
else ifeq (arm64-v8a,$(TARGET_ARCH_ABI))
LOCAL_CFLAGS +=
else ifeq (x86,$(TARGET_ARCH_ABI))
LOCAL_CFLAGS += -mf16c
LOCAL_CFLAGS += -mfma
LOCAL_CFLAGS += -mavx2
else ifeq (x86_64,$(TARGET_ARCH_ABI))
LOCAL_CFLAGS += -mf16c
LOCAL_CFLAGS += -mfma
LOCAL_CFLAGS += -mavx2
else
LOCAL_CFLAGS +=
endif

LOCAL_CFLAGS += -Wall
LOCAL_CFLAGS += -Werror=return-type

LOCAL_CFLAGS += -DPAL_STDCPP_COMPAT=1
LOCAL_CFLAGS += -DMP_EXPORT=

LOCAL_C_INCLUDES :=
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../CoreRT/src/Native/inc/unix
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../DirectXMath/Inc
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../OpenCV/modules/core/include
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../OpenCV/modules/imgproc/include
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../OpenCV/modules/imgcodecs/include
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../OpenCV/modules/videoio/include
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../thirdparty/mediapipe/include
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../thirdparty/OpenCL-Headers

LOCAL_CPPFLAGS := 
LOCAL_CPPFLAGS += -std=c++20

LOCAL_LDFLAGS :=
LOCAL_LDFLAGS += -Wl,--enable-new-dtags
LOCAL_LDFLAGS += -Wl,-rpath,\$$ORIGIN
LOCAL_LDFLAGS += -Wl,--version-script,$(LOCAL_PATH)/BRX-Motion.map

LOCAL_LDFLAGS += -ldl

LOCAL_STATIC_LIBRARIES :=
LOCAL_STATIC_LIBRARIES += OpenCV

LOCAL_SHARED_LIBRARIES :=
LOCAL_STATIC_LIBRARIES += BRX-Motion-MediaPipe-Model-Asset
LOCAL_SHARED_LIBRARIES += mediapipe
LOCAL_SHARED_LIBRARIES += OpenCL
LOCAL_SHARED_LIBRARIES += BRX-Physics-BT
LOCAL_SHARED_LIBRARIES += BRX-WSI
LOCAL_SHARED_LIBRARIES += McRT-Malloc
LOCAL_SHARED_LIBRARIES += FFmpeg-AVCodec
LOCAL_SHARED_LIBRARIES += FFmpeg-AVFormat
LOCAL_SHARED_LIBRARIES += FFmpeg-AVUtil
LOCAL_SHARED_LIBRARIES += FFmpeg-SWResample
LOCAL_SHARED_LIBRARIES += FFmpeg-SWScale

include $(BUILD_SHARED_LIBRARY)
