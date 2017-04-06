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

#include "VisionCorrection.h"

#include "CameraIntrinsic.h"
#include "ImageFrame.h"
#include "VisionTracker.h"
#include "Mapping.h"

#include "LogStream.h"

using namespace std;

extern "C"{

struct timeval tstart;
struct timeval tbegin;
struct timeval tend;

static bool isFirstUpdate = true;
static int imuCount = 0;

static bool isFirstImage = true;
CVD::Image<CVD::byte> imageData;
CVD::ImageRef SBISize;
TooN::SO3<> pose_reset;
float state_reset[7];
int frameCount = 0;

SmallBlurryImage SBI;
SmallBlurryImage SI;
int countFromCorrection = 0;

float fx, fy, cx, cy, d;

MyStreamBuf g_MyStreamBuf;

/*
* Code for imu pose.
*/

// Init imu pose
void InitIMU(float* pimuval, float* q)
{
    MahonyAHRS::init(pimuval[0], pimuval[1], pimuval[2], q);
}

// Reset
JNIEXPORT void JNICALL
Java_com_citrus_slam_MahonyAHRS_QuaternionSensor_nativeReset( JNIEnv* env, jobject thiz)
{
    __android_log_print(ANDROID_LOG_INFO, "JNIMsg", "JNI nativeReset called");
    MahonyAHRS::reset();
    isFirstUpdate = true;
    isFirstImage = true;
    imuCount = 0;
    frameCount = 0;
    countFromCorrection = 0;

    std::cout.rdbuf(&g_MyStreamBuf);
    //NOTE: std::endl会立即调用sync方法将缓冲区字符写入log，并不只是换行用
    std::cout << "hello, I'm c log ! " << 123 << std::endl;
}

// Update By IMU
JNIEXPORT void JNICALL
Java_com_citrus_slam_MahonyAHRS_QuaternionSensor_nativeUpdateIMU( JNIEnv* env, jobject thiz,
        jfloatArray imuval, jfloatArray q)
{

    float* pimuval = env->GetFloatArrayElements(imuval, 0);
    float* pq = env->GetFloatArrayElements(q, 0);

    float imufreq = 50.f;      // the frequency is important

    if (isFirstUpdate) {        // if first update imu, init it
        gettimeofday(&tstart, 0);
        gettimeofday(&tbegin, 0);
        isFirstUpdate = false;
        InitIMU(pimuval, pq);
        __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                "JNI Init IMU called, init q : %f %f %f %f", pq[0], pq[1], pq[2], pq[3]);
    } else {

        gettimeofday(&tend, 0);
        imufreq = 1000000u/ (((tend.tv_sec - tstart.tv_sec)*1000000u
                + tend.tv_usec - tstart.tv_usec));        // caculate the frequency
        gettimeofday(&tstart, 0);
    }

    MahonyAHRS::updateIMU(pimuval[3], pimuval[4], pimuval[5], pimuval[0], pimuval[1], pimuval[2],
            imufreq, pq[0], pq[1], pq[2], pq[3]);

    imuCount++;
    if (imuCount==500) {
        __android_log_print(ANDROID_LOG_INFO, "JNIMsg", "JNI nativeUpdateIMU called,"
                "imufreq: %f, q : %f %f %f %f", imufreq, pq[0], pq[1], pq[2], pq[3]);
        imuCount=0;
    }

    env->ReleaseFloatArrayElements(imuval, pimuval, 0);
    env->ReleaseFloatArrayElements(q, pq, 0);
}

/*
* Following code is for vision correction.
*/

// Reset Vision
JNIEXPORT void JNICALL
Java_com_citrus_slam_MahonyAHRS_QuaternionSensor_nativeResetVision( JNIEnv* env, jobject thiz)
{
    isFirstImage = true;
}

// Update by vision, 3D-2D feature Pose based.
JNIEXPORT void JNICALL
Java_com_citrus_slam_MahonyAHRS_QuaternionSensor_nativeUpdateVision3D2D( JNIEnv* env, jobject thiz,
        jbyteArray imageArray, jint width, jint height)
{

    if (isFirstUpdate) return;      // if has not do imu update,
                                    // return

    // static vars
    static TooN::SO3<> R_ic = loadIMUExtrinsic();
    static std::vector< cv::Point2f > pt_1;
    static cv::Mat K = loadCameraIntrinsic(width, height);
    static CameraIntrinsic K1(K.at<double>(2)  / 8, K.at<double>(3) / 8,
            K.at<double>(0) / 8, K.at<double>(1) / 8, 0, 0, width / 8, height / 8 );
    static Mapping map(&K1);
    static VisionTracker tracker(&K1, &map);

    // get image array data
    int len = env->GetArrayLength(imageArray);
    imageData.resize(CVD::ImageRef(width, height));
    env->GetByteArrayRegion(imageArray, 0, width*height, (jbyte*)imageData.data() );

    // OpenCV Mat
    cv::Mat cvImage(height, width, CV_8UC1);
    memcpy(cvImage.data, imageData.data(), width*height);

    static ATANCamera mCamera("/sdcard/calibration/calibration.txt");

    if (isFirstImage) {                     // if first image, setup correction SBI
        // create SBI
        SBI = SmallBlurryImage(imageData);
        //SBI.MakeJacs();
        SBISize = SBI.GetSize();
        __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                "JNI nativeUpdateVision called,"
                "SBISize: %d, %d",
                SBISize.x, SBISize.y);

        // record reset pose
        MahonyAHRS::getState(state_reset);
        pose_reset = TooN::SO3<>::exp(
                        WFromQ(TooN::makeVector(state_reset[0], state_reset[1],
                            state_reset[2], state_reset[3]))
                        );

        // add first frame
        tracker.reset();
        cv::Mat img_2;
        cv::resize(cvImage, img_2, cv::Size(width/8, height/8));
        ImageFrame newFrame(img_2, &K1);
        tracker.TrackMonocular(newFrame);

        // switch ifFirstImage
        isFirstImage = false;
    } else {                                // if new image, do correction routine

        SI = SmallBlurryImage(imageData);
        double diffVal = SI.ZMSSD(SBI);     // compute SSD diff

        if (frameCount == 200) {
            __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                    "JNI nativeUpdateVision called,"
                    "diffVal: %f", diffVal);
            frameCount = 0;
        }

        if (tracker.state==tracker.INITIALIZING) {
            cv::Mat img_2;
            cv::resize(cvImage, img_2, cv::Size(width/8, height/8));
            ImageFrame newFrame(img_2, &K1);
            tracker.TrackMonocular(newFrame);
        } else if (diffVal < 50000 && countFromCorrection > 40) {              // if SSD small , do correction
            cv::Mat img_2;
            cv::resize(cvImage, img_2, cv::Size(width/8, height/8));
            ImageFrame newFrame(img_2, &K1);
            if (tracker.TrackMonocular(newFrame)) {

                TooN::SO3<> R_correction;
                std::stringstream ss;
                ss  << tracker.mR.at<double>(0, 0) << " "
                    << tracker.mR.at<double>(0, 1) << " "
                    << tracker.mR.at<double>(0, 2) << " "
                    << tracker.mR.at<double>(1, 0) << " "
                    << tracker.mR.at<double>(1, 1) << " "
                    << tracker.mR.at<double>(1, 2) << " "
                    << tracker.mR.at<double>(2, 0) << " "
                    << tracker.mR.at<double>(2, 1) << " "
                    << tracker.mR.at<double>(2, 2);
                ss >> R_correction;

                std::cout << "R_correction:" << std::endl;
                std::cout << R_correction << std::endl;

                TooN::SO3<> pose_correction =
                        pose_reset * R_ic.inverse() * R_correction * R_ic ;        // corrected pose
                TooN::Vector<4> state_correction =
                        QFromW(pose_correction.ln());           // convert to Quaternion

                float state_now[7];
                state_now[0] = state_correction[0];
                state_now[1] = state_correction[1];
                state_now[2] = state_correction[2];
                state_now[3] = state_correction[3];
                state_now[4] = 0;
                state_now[5] = 0;
                state_now[6] = 0;

                MahonyAHRS::setState(state_now);                // set corrected state

                countFromCorrection = 0;
                __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                                "JNI nativeUpdateVision called,"
                                "Reset State, diffVal: %f", diffVal);
            }
        }
    }

    frameCount++;
    countFromCorrection++;

    if (countFromCorrection == 400) {               // if too long time from last correction, reset
                                                    // correction
        isFirstImage = true;
        __android_log_print(ANDROID_LOG_INFO, "mylog",
                "JNI nativeUpdateVision,"
                "CountFromCorrection too long,"
                "Reset SBI");
    }

}

// Update by vision, 2D-2D feature Pose based.
JNIEXPORT void JNICALL
Java_com_citrus_slam_MahonyAHRS_QuaternionSensor_nativeUpdateVision2D2D( JNIEnv* env, jobject thiz,
        jbyteArray imageArray, jint width, jint height)
{

    if (isFirstUpdate) return;      // if has not do imu update,
                                    // return
    // static vars
    static TooN::SO3<> R_ic = loadIMUExtrinsic();
    static cv::Mat K = loadCameraIntrinsic(width, height);
    static std::vector< cv::Point2f > pt_1;
    static cv::Mat img_1, img_2;

    // get image array data
    int len = env->GetArrayLength(imageArray);
    imageData.resize(CVD::ImageRef(width, height));
    env->GetByteArrayRegion(imageArray, 0, width*height, (jbyte*)imageData.data() );

    // OpenCV Mat
    static cv::Mat cvImage(height, width, CV_8UC1);
    memcpy(cvImage.data, imageData.data(), width*height);

    static ATANCamera mCamera("/sdcard/calibration/calibration.txt");

    if (isFirstImage) {                     // if first image, setup correction SBI
        // create SBI
        SBI = SmallBlurryImage(imageData);
        //SBI.MakeJacs();
        SBISize = SBI.GetSize();
        __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                "JNI nativeUpdateVision called,"
                "SBISize: %d, %d",
                SBISize.x, SBISize.y);

        // record reset pose
        MahonyAHRS::getState(state_reset);
        pose_reset = TooN::SO3<>::exp(
                        WFromQ(TooN::makeVector(state_reset[0], state_reset[1],
                            state_reset[2], state_reset[3]))
                        );

        // detect fast feature
        pt_1.resize(0);
        std::vector< cv::KeyPoint > kps;
        cv::resize(cvImage, img_2, cv::Size(width/8, height/8));
        img_2.copyTo(img_1);
        cv:FAST(img_1, kps, 40);
        pt_1.reserve(kps.size());
        for (int i = 0, _end = (int)kps.size(); i < _end; i++) {
            pt_1.push_back(kps[i].pt);
        }
        __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                        "JNI nativeUpdateVision called,"
                        "FAST feature number: %d",
                        (int)kps.size());

        // switch ifFirstImage
        isFirstImage = false;
    } else {                                // if new image, do correction routine
        SI = SmallBlurryImage(imageData);
        double diffVal = SI.ZMSSD(SBI);     // compute SSD diff

        if (frameCount == 200) {
            __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                    "JNI nativeUpdateVision called,"
                    "diffVal: %f", diffVal);
            frameCount = 0;
        }

        if (diffVal < 50000 && countFromCorrection > 40) {              // if SSD small , do correction
            float state_now[7];
            MahonyAHRS::getState(state_now);
                   // rotation from SBI to SI

            cv::resize(cvImage, img_2, cv::Size(width/8, height/8));
            TooN::SO3<> R_correction = Roation2d2d(img_1, img_2, pt_1 , K);

            TooN::SO3<> pose_correction =
                    pose_reset * R_ic.inverse() * R_correction.inverse() * R_ic ;        // corrected pose
            TooN::Vector<4> state_correction =
                    QFromW(pose_correction.ln());           // convert to Quaternion

            state_now[0] = state_correction[0];
            state_now[1] = state_correction[1];
            state_now[2] = state_correction[2];
            state_now[3] = state_correction[3];
            state_now[4] = 0;
            state_now[5] = 0;
            state_now[6] = 0;

            MahonyAHRS::setState(state_now);                // set corrected state

            countFromCorrection = 0;
            __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                            "JNI nativeUpdateVision called,"
                            "Reset State, diffVal: %f", diffVal);

        }

    }

    frameCount++;
    countFromCorrection++;

    if (countFromCorrection == 400) {               // if too long time from last correction, reset
                                                    // correction
        isFirstImage = true;
        __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                "JNI nativeUpdateVision,"
                "CountFromCorrection too long,"
                "Reset SBI");
    }
}

// Update by vision, SBI Based
JNIEXPORT void JNICALL
Java_com_citrus_slam_MahonyAHRS_QuaternionSensor_nativeUpdateVisionSBI( JNIEnv* env, jobject thiz,
        jbyteArray imageArray, jint width, jint height)
{

    if (isFirstUpdate) return;      // if has not do imu update,
                                    // return
    static TooN::SO3<> R_ic = loadIMUExtrinsic();

    // get image array data
    int len = env->GetArrayLength(imageArray);
    imageData.resize(CVD::ImageRef(width, height));
    env->GetByteArrayRegion(imageArray, 0, width*height, (jbyte*)imageData.data() );

    // test OpenCV Mat
    cv::Mat cvImage(height, width, CV_8UC1);

    static ATANCamera mCamera("/sdcard/calibration/calibration.txt");

    if (isFirstImage) {                     // if first image, setup correction SBI
        SBI = SmallBlurryImage(imageData);
        SBI.MakeJacs();
        MahonyAHRS::getState(state_reset);
        pose_reset = TooN::SO3<>::exp(
                        WFromQ(TooN::makeVector(state_reset[0], state_reset[1],
                            state_reset[2], state_reset[3]))
                        );

        SBISize = SBI.GetSize();
        __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                "JNI nativeUpdateVision called,"
                "SBISize: %d, %d",
                SBISize.x, SBISize.y);

        isFirstImage = false;
    } else {                                // if new image, do correction routine
        SI = SmallBlurryImage(imageData);
        double diffVal = SI.ZMSSD(SBI);     // compute SSD diff

        if (frameCount == 200) {
            __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                    "JNI nativeUpdateVision called,"
                    "diffVal: %f", diffVal);
            frameCount = 0;
        }

        if (diffVal < 50000) {              // if SSD small , do correction
            float state_now[7];
            MahonyAHRS::getState(state_now);
            std::pair< TooN::SO3<>, double > rotation_pair = CalcSBIRotation(SBI, SI);       // rotation from SBI to SI

            if (rotation_pair.second < 15000) {
                TooN::SO3<> pose_correction =
                        pose_reset * R_ic.inverse() * rotation_pair.first.inverse() * R_ic ;        // corrected pose
                TooN::Vector<4> state_correction =
                        QFromW(pose_correction.ln());           // convert to Quaternion

                state_now[0] = state_correction[0];
                state_now[1] = state_correction[1];
                state_now[2] = state_correction[2];
                state_now[3] = state_correction[3];
                state_now[4] = 0;
                state_now[5] = 0;
                state_now[6] = 0;

                MahonyAHRS::setState(state_now);                // set corrected state

                countFromCorrection = 0;
                __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                                "JNI nativeUpdateVision called,"
                                "Reset State, diffVal: %f, error: %f", diffVal, rotation_pair.second);
            }
        }

    }

    frameCount++;
    countFromCorrection++;

    if (countFromCorrection == 400) {               // if too long time from last correction, reset
                                                    // correction
        isFirstImage = true;
        __android_log_print(ANDROID_LOG_INFO, "JNIMsg",
                "JNI nativeUpdateVision,"
                "CountFromCorrection too long,"
                "Reset SBI");
    }
}

}
