#
# Copyright 2014 Google Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

LOCAL_PATH := $(call my-dir)
PROJECT_ROOT_FROM_JNI:= ../../../../..
PROJECT_ROOT:= $(call my-dir)/../../../../..

SUPERPOWERED_PATH := $(PROJECT_ROOT)/Superpowered

include $(CLEAR_VARS)
LOCAL_MODULE := Superpowered

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
	LOCAL_SRC_FILES := $(SUPERPOWERED_PATH)/libSuperpoweredAndroidarmeabi-v7a.a
else
	ifeq ($(TARGET_ARCH_ABI),arm64-v8a)
		LOCAL_SRC_FILES := $(SUPERPOWERED_PATH)/libSuperpoweredAndroidarm64-v8a.a
	else
		ifeq ($(TARGET_ARCH_ABI),x86_64)
			LOCAL_SRC_FILES := $(SUPERPOWERED_PATH)/libSuperpoweredAndroidX86_64.a
		else
			LOCAL_SRC_FILES := $(SUPERPOWERED_PATH)/libSuperpoweredAndroidX86.a
		endif
	endif
endif

include $(PREBUILT_STATIC_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE    := libcpp_rgb_depth_sync_example
LOCAL_STATIC_LIBRARIES := Superpowered
LOCAL_SHARED_LIBRARIES := tango_client_api tango_support
LOCAL_CFLAGS    := -std=c++11

LOCAL_C_INCLUDES := $(PROJECT_ROOT)/tango-service-sdk/include/ \
                    $(PROJECT_ROOT)/tango_gl/include \
                    $(PROJECT_ROOT)/third_party/glm/ \
                    $(SUPERPOWERED_PATH)


LOCAL_SRC_FILES := camera_texture_drawable.cc \
                   color_image.cc \
                   depth_image.cc \
                   jni_interface.cc \
                   rgbdsegmentation_application.cc \
                   scene.cc \
                   util.cc \
                   Segmentacion.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/bounding_box.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/camera.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/conversions.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/cube.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/drawable_object.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/frustum.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/grid.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/line.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/mesh.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/shaders.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/trace.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/transform.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/src/util.cc \
                   AndroidIO/SuperpoweredAndroidAudioIO.cpp

LOCAL_LDLIBS    := -llog -lGLESv2 -L$(SYSROOT)/usr/lib -lOpenSLES
include $(BUILD_SHARED_LIBRARY)
$(call import-add-path, $(PROJECT_ROOT))
$(call import-module,tango_client_api)
$(call import-module,tango_support)


