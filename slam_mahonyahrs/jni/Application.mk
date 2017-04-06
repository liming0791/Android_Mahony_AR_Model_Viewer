APP_STL := gnustl_shared
#APP_STL := c++_static
#APP_STL := stlport_static
NDK_TOOLCHAIN_VERSION := 4.9
APP_CPPFLAGS := -frtti -fexceptions -mfloat-abi=softfp -O3 -Ofast -ffast-math -ftree-vectorize -mfpu=neon -std=gnu++0x -Wno-deprecated -pthread -std=c++11
#APP_ABI := armeabi armeabi-v7a
APP_ABI := armeabi-v7a
APP_PLATFORM := android-21
APP_OPTIM := release
#APP_OPTIM := debug
