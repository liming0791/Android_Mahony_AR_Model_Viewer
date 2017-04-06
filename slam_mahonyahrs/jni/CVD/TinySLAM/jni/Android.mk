# Copyright (C) 2009 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.ccrg/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OpenCV_INSTALL_MODULES := on
OpenCV_CAMERA_MODULES := off
OPENCV_LIB_TYPE :=STATIC
include CVD/opencv3/jni/OpenCV.mk

LOCAL_CPP_EXTENSION := .cc .cpp .cxx
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../include
LOCAL_MODULE    := TinySLAM

LOCAL_CFLAGS += -I$(LOCAL_PATH)/../../g2o/include   \
    -I$(LOCAL_PATH)/../../boost_1_53_0/include      \
    -pthread

LOCAL_SRC_FILES += ../src/CameraIntrinsic.cc                      \
../src/Converter.cc                                               \
../src/ImageFrame.cc                                              \
../src/Mapping.cc                                                 \
../src/MedianFilter.cc                                            \
../src/Timer.cc                                                   \
../src/VisionTracker.cc

LOCAL_STATIC_LIBRARIES += TooN
LOCAL_STATIC_LIBRARIES += eigen
LOCAL_STATIC_LIBRARIES += cvd
LOCAL_STATIC_LIBRARIES += CSparse
LOCAL_STATIC_LIBRARIES += cpufeatures
LOCAL_SHARED_LIBRARIES += g2o_cli               g2o_core                   g2o_csparse_extension \
                          g2o_ext_csparse       g2o_interface              g2o_parser \
                          g2o_solver_csparse    g2o_solver_dense           g2o_solver_eigen \
                          g2o_solver_pcg        g2o_solver_slam2d_linear   g2o_solver_structure_only \
                          g2o_stuff             g2o_types_data             g2o_types_icp \
                          g2o_types_sba         g2o_types_sclam2d          g2o_types_sim3 \
                          g2o_types_slam2d      g2o_types_slam2d_addons    g2o_types_slam3d \
                          g2o_types_slam3d_addons

LOCAL_LDLIBS += -landroid -llog

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_C_INCLUDES)          #export includes
LOCAL_EXPORT_LDLIBS := $(LOCAL_LDLIBS)                  #export linker cmds
LOCAL_EXPORT_CFLAGS := $(LOCAL_CFLAGS)                  #export c flgs
LOCAL_EXPORT_CPPFLAGS := $(LOCAL_CPPFLAGS)              #export cpp flgs
LOCAL_EXPORT_CXXFLAGS := $(LOCAL_CXXFLAGS)              #export cpp flgs

# include $(BUILD_SHARED_LIBRARY)
include $(BUILD_SHARED_LIBRARY)

$(call import-add-path,$(LOCAL_PATH)/../../)
$(call import-module, TooN)
$(call import-module, eigen)
$(call import-module, cvd)
$(call import-module, CSparse)
$(call import-module, g2o)

$(call import-module,android/cpufeatures)
