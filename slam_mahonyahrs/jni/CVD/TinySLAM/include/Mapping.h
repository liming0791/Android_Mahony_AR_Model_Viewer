#ifndef MAPPING_H
#define MAPPING_H

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <set>

#include <opencv2/opencv.hpp>

#include "ImageFrame.h"
#include "CameraIntrinsic.h"

using namespace std;

class Mapping
{
    public:
        CameraIntrinsic* K;
        std::set< cv::Point3f* > mapPoints;
        std::vector< ImageFrame* > keyFrames;

        Mapping() = default;
        Mapping(CameraIntrinsic* _K):K(_K){};
        void InitMap(ImageFrame& lf, ImageFrame& rf);
        void ClearMap();
};

#endif
