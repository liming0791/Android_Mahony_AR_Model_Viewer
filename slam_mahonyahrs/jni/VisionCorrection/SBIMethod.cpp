//
// Created by liming on 17-3-30.
//

#include <sys/time.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <android/log.h>
#include <jni.h>

#include <cvd/image.h>
#include <cvd/image_io.h>
#include <cvd/byte.h>
#include <cvd/utility.h>
#include <cvd/convolution.h>
#include <cvd/vision.h>

#include "SmallBlurryImage.h"
#include "ATANCamera.h"

#include "VisionCorrection.h"

std::pair< TooN::SO3<>, double> CalcSBIRotation(SmallBlurryImage& SBI,
                                                SmallBlurryImage& SI)
{
    static ATANCamera mCamera("/sdcard/calibration/calibration.txt");   // to be inited from file
    std::pair<TooN::SE2<>, double> result_pair;
    result_pair = SI.IteratePosRelToTarget(SBI, 10);
    TooN::SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCamera);
    //__android_log_print(ANDROID_LOG_INFO, "JNIMsg",
    //                   "JNI CalcSBIRotation called,"
    //                    "error: %f",
    //                    result_pair.second);
    return std::pair< TooN::SO3<>, double >(se3Adjust.get_rotation(), result_pair.second);
    //return std::pair< TooN::SO3<>, double >(TooN::SO3<>(), 3000000);
}