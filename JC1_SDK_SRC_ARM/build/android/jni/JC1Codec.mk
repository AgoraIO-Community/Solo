LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

SRC_PATH := ../../../src/evrc/src/

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/../../../interface \
	$(LOCAL_PATH)/../../../src \
	$(LOCAL_PATH)/../../../src/libBWE \
	$(LOCAL_PATH)/../../../src/libSATECodec
	
LOCAL_SRC_FILES := $(wildcard ../../../src/*.c)  \
		$(wildcard ../../../src/libBWE/*.c)  \
		$(wildcard ../../../src/libSATECodec/*.c)  

LOCAL_CFLAGS += -DNO_ASM

LOCAL_MODULE := libJC1Codec

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
    LOCAL_CFLAGS += -DHAVE_NEON=1
endif
LOCAL_CFLAGS += $(GLOBAL_CFLAGS)

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
LOCAL_ARM_MODE := arm
LOCAL_CFLAGS += -marm -O3 -fPIC
endif

ifeq ($(TARGET_ARCH_ABI),armeabi)
LOCAL_ARM_MODE := arm
LOCAL_CFLAGS += -marm -O3
endif

ifeq ($(TARGET_ARCH_ABI),x86)
LOCAL_CFLAGS += -march=i686 -O3
endif

ifeq ($(TARGET_ARCH_ABI),mips)
LOCAL_CFLAGS += -O3 -fPIC

LOCAL_CFLAGS += -DOPT_GENERIC -DREAL_IS_FLOAT
endif

include $(BUILD_STATIC_LIBRARY)