//
// Created by liming on 17-3-30.
//

#ifndef ANDROID_3D_MODEL_VIEWER_MASTER_VISIONCORRECTION_H
#define ANDROID_3D_MODEL_VIEWER_MASTER_VISIONCORRECTION_H

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

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "MahonyAHRS.h"
#include "SmallBlurryImage.h"
#include "ATANCamera.h"

// declarations
TooN::Vector<4> QFromW(const TooN::Vector<3> & w);
TooN::Vector<3> WFromQ(const TooN::Vector<4> & q);
TooN::SO3<> loadIMUExtrinsic();
cv::Mat loadCameraIntrinsic(int w, int h);
std::pair< TooN::SO3<>, double> CalcSBIRotation(
                                    SmallBlurryImage& SBI,
                                    SmallBlurryImage& SI);
SO3<> Roation2d2d(cv::Mat & img_1, cv::Mat & img_2, std::vector< cv::Point2f > & pt_1, cv::Mat & K);


#endif //ANDROID_3D_MODEL_VIEWER_MASTER_VISIONCORRECTION_H
