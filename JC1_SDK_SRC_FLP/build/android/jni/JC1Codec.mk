LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

SRC_PATH := ../../../src/evrc/src/

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/../../../interface 
	
LOCAL_SRC_FILES := $(wildcard ../../../src/*.c)  \
		$(wildcard ../../../src/libBWE/*.c)  \
		$(wildcard ../../../src/libSATECodec/*.c)  


LOCAL_MODULE := libJC1Codec
ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
    LOCAL_CFLAGS += -DHAVE_NEON=1
endif
LOCAL_CFLAGS += $(GLOBAL_CFLAGS)

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
LOCAL_ARM_MODE := arm
LOCAL_CFLAGS += -marm -O3 
endif

ifeq ($(TARGET_ARCH_ABI),armeabi)
LOCAL_ARM_MODE := arm
LOCAL_CFLAGS += -marm -O3 
endif

ifeq ($(TARGET_ARCH_ABI),x86)
LOCAL_CFLAGS += -march=i686 -O3 
endif
# include $(BUILD_STATIC_LIBRARY)
include $(BUILD_SHARED_LIBRARY)
