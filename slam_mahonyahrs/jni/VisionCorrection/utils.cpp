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

#include <TooN/so3.h>
#include <TooN/se3.h>

#include <opencv2/opencv.hpp>

#include "VisionCorrection.h"

TooN::Vector<4> QFromW(const TooN::Vector<3> & w) {
    double rad = sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);
    double cos_rad_2 = cos(rad/2);
    double sin_rad_2 = sin(rad/2);

    return TooN::makeVector(cos_rad_2, w[0]/rad*sin_rad_2, w[1]/rad*sin_rad_2, w[2]/rad*sin_rad_2);
}

TooN::Vector<3> WFromQ(const TooN::Vector<4> & q) {
    double rad = acos(q[0])*2;
    double len = sqrt(q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    return TooN::makeVector(rad*q[1]/len, rad*q[2]/len, rad*q[3]/len );
}

TooN::SO3<> loadIMUExtrinsic() {
    TooN::SE3<> res;
    std::ifstream file("/sdcard/calibration/IMUExtrinsic.txt");

    if (!file.is_open()) {
        __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                           "JNI loadIMUExtrinsic called,"
                            "warning: open IMUExtrinsic.txt failed !");
        return res.get_rotation();
    }
    std::string line;
    std::getline(file, line);
    std::stringstream ss(line);
    ss >> res;

    std::stringstream outlog;
    outlog << res;

    __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                               "JNI loadIMUExtrinsic called,"
                                "SE3: %s", outlog.str().c_str());

    return res.get_rotation();
}

cv::Mat loadCameraIntrinsic(int w, int h) {
    std::ifstream file("/sdcard/calibration/calibration.txt");
    if (!file.is_open()) {
        __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                                   "JNI loadCameraIntrinsic called,"
                                    "warning: open calibration.txt failed !");
        return cv::Mat(4, 1, CV_64FC1);
    }

    double fx, fy, cx, cy, d;
    std::string line;
    getline(file, line);
    std::stringstream ss(line);
    ss >> fx >> fy >> cx >> cy >> d;

    __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                           "JNI loadCameraIntrinsic called,"
                            " %f, %f, %f, %f, %f",  fx,  fy,  cx, cy, d);

    return (cv::Mat_<double>(4,1) << fx*w, fy*h, cx*w, cy*h );
}