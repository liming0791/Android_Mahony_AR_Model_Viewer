LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_cli
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_cli.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_core
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_core.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_csparse_extension
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_csparse_extension.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_ext_csparse
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_ext_csparse.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_interface
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_interface.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_parser
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_parser.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_solver_csparse
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_solver_csparse.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_solver_dense
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_solver_dense.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_solver_eigen
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_solver_eigen.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_solver_pcg
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_solver_pcg.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_solver_slam2d_linear
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_solver_slam2d_linear.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_solver_structure_only
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_solver_structure_only.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_stuff
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_stuff.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_types_data
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_types_data.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_types_icp
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_types_icp.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_types_sba
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_types_sba.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_types_sclam2d
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_types_sclam2d.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_types_sim3
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_types_sim3.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_types_slam2d
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_types_slam2d.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_types_slam2d_addons
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_types_slam2d_addons.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_types_slam3d
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_types_slam3d.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_types_slam3d_addons
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_SRC_FILES := $(LOCAL_PATH)/../armeabi-v7a/lib/libg2o_types_slam3d_addons.so
include $(PREBUILT_SHARED_LIBRARY)