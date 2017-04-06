//
// Created by liming on 17-3-31.
//

#include <android/log.h>

#ifndef ANDROID_3D_MODEL_VIEWER_MASTER_REDIRECTPRINTF_H
#define ANDROID_3D_MODEL_VIEWER_MASTER_REDIRECTPRINTF_H

#define printf(...) __android_log_print(ANDROID_LOG_DEBUG, "mylog", __VA_ARGS__);

#endif //ANDROID_3D_MODEL_VIEWER_MASTER_REDIRECTPRINTF_H
