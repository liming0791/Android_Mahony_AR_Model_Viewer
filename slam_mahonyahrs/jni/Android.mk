LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

OpenCV_INSTALL_MODULES := on
OpenCV_CAMERA_MODULES := off
OPENCV_LIB_TYPE :=STATIC
include CVD/opencv3/jni/OpenCV.mk

LOCAL_CPP_EXTENSION := .cpp
LOCAL_CFLAGS += -std=c++11
LOCAL_CPPFLAGS += -std=c++11
LOCAL_MODULE    := MahonyAHRS

LOCAL_C_INCLUDES += $(LOCAL_PATH)/MahonyAHRS
LOCAL_C_INCLUDES += $(LOCAL_PATH)/VisionCorrection

AHRS_PATH := ./MahonyAHRS
LOCAL_SRC_FILES += \
$(AHRS_PATH)/MahonyAHRS.cpp             \
./VisionCorrection/Pose2D2DMethod.cpp   \
./VisionCorrection/SBIMethod.cpp       \
./VisionCorrection/Pose3D2DMethod.cpp   \
./VisionCorrection/utils.cpp            \
./mahony_ahrs_main.cpp


LOCAL_STATIC_LIBRARIES += cvd
LOCAL_STATIC_LIBRARIES += SBI
LOCAL_SHARED_LIBRARIES += TinySLAM
LOCAL_LDLIBS    += -llog -landroid

LOCAL_CFLAGS += -g

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_C_INCLUDES) #export includes
LOCAL_EXPORT_LDLIBS := $(LOCAL_LDLIBS) #export linker cmds
LOCAL_EXPORT_CFLAGS := $(LOCAL_CFLAGS) #export c flgs
LOCAL_EXPORT_CPPFLAGS := $(LOCAL_CPPFLAGS) #export cpp flgs
LOCAL_EXPORT_CXXFLAGS := $(LOCAL_CXXFLAGS) #export cpp flgs

include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(LOCAL_PATH)/CVD)
$(call import-module, cvd)
$(call import-module, SBI)
$(call import-module, TinySLAM)

$(call import-module,android/cpufeatures)





