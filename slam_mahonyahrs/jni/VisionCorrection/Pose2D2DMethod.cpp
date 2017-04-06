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

#include <TooN/TooN.h>
#include <TooN/so3.h>
#include <TooN/se3.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "VisionCorrection.h"

TooN::SO3<> Roation2d2d(cv::Mat & img_1, cv::Mat & img_2,
        std::vector< cv::Point2f > & pt_1, cv::Mat & K) {

    std::vector< cv::Point2f > pt_2;
    cv::Mat status, err;
    cv::calcOpticalFlowPyrLK(img_1, img_2, pt_1, pt_2, status, err) ;

    // homography estimation cut outliners
    cv::Mat inliner;
    std::vector< cv::Point2f > pt_11, pt_22;
    pt_11.reserve(pt_1.size());
    pt_22.reserve(pt_2.size());
    cv::findHomography(pt_1, pt_2, cv::RANSAC, 3, inliner);
    for (int i = 0, _end = (int)pt_1.size(); i < _end; i++) {
        if (inliner.at<unsigned char>(i) == 1) {
            pt_11.push_back(pt_1[i]);
            pt_22.push_back(pt_2[i]);
        }
    }

    // calculate essential matrix
    cv::Point2d principal_point ( K.at<double>(2)/8, K.at<double>(3)/8 );   //相机光心, TUM dataset标定值
    double focal_length = (K.at<double>(0) + K.at<double>(0)) / 16;          //相机焦距, TUM dataset标定值
    cv::Mat essential_matrix
            = cv::findEssentialMat ( pt_11, pt_22, focal_length, principal_point );
    //cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    // recover pose from essential matrix
    cv::Mat R, t;
    cv::recoverPose ( essential_matrix, pt_11, pt_22, R, t, focal_length, principal_point );

    TooN::SO3<> res;
    std::stringstream data;
    data << R.at<double>(0,0) << " " << R.at<double>(0,1) << " " << R.at<double>(0,2) << " "
        << R.at<double>(1,0) << " " << R.at<double>(1,1) << " " << R.at<double>(1,2) << " "
        << R.at<double>(2,0) << " " << R.at<double>(2,1) << " " << R.at<double>(2,2);
    data >> res;

    __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
            "JNI Roation2d2d called,"
            "focal_length: %f, cx: %f, cy: %f",
            focal_length, principal_point.x, principal_point.y);

    {
        std::stringstream outlog;
        outlog << res;
        __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                       "JNI Roation2d2d called,"
                        "SO3: %s", outlog.str().c_str());
    }

    {
        std::stringstream outlog;
                outlog << R;
                __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                       "JNI Roation2d2d called,"
                        "CV R: %s", outlog.str().c_str());
    }

    return res;

}