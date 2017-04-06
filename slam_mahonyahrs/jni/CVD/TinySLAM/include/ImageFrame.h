#ifndef IMAGEFRAME_H
#define IMAGEFRAME_H

#include <stdio.h>
#include <stdlib.h>

#include <iostream>

#include <boost/bimap.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <TooN/se3.h>
#include <TooN/so3.h>

#include <cvd/image.h>
#include <cvd/image_io.h>
#include <cvd/vision.h>
#include <cvd/esm.h>

#include "CameraIntrinsic.h"

using namespace std;

typedef boost::bimap< int, cv::Point3f* > Map_2d_3d;
typedef Map_2d_3d::value_type Map_2d_3d_key_val;

class ImageFrame
{
    public:
        ImageFrame() = default;
        ~ImageFrame() {};
        explicit ImageFrame(const cv::Mat& frame, CameraIntrinsic* _K);
        explicit ImageFrame(const ImageFrame& imgFrame);

        void extractFAST();
        void opticalFlowFAST(ImageFrame& refFrame);
        void opticalFlowTrackedFAST(ImageFrame& lastFrame);
//        void SBITrackFAST(ImageFrame& refFrame);

        cv::Mat GetTwcMat();
        cv::Mat GetTcwMat();

        cv::Mat image;                         // image data
//        CVD::Image<CVD::byte> mSBI;         // small blurry image
        vector< cv::KeyPoint > keyPoints;      // original fast keypoints
        vector< cv::Point2f > points;          // Just point2f
        vector< cv::Point2f > undisPoints;     // Undistorted keypoints

        vector< cv::Point2f > trackedPoints;
        vector< cv::Point2f > undisTrackedPoints;

        Map_2d_3d map_2d_3d;

        TooN::SE3<> mTcw;                      // Transformation from w to camera
        cv::Mat R, t;                          // Tcw R, t
        ImageFrame* mRefFrame;                 // Reference Frame
        CameraIntrinsic* K;                    // CameraIntrinsic
};

#endif
