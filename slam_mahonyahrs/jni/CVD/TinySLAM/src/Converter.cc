#include "Converter.h"

#include "RedirectPrintf.h"

namespace Converter
{

    void TooNSO3_Mat(const TooN::SO3<>& so3, cv::Mat& mat)
    {
        if (mat.empty() || mat.rows != 3 || mat.cols != 3)
            mat = cv::Mat(3, 3, CV_64FC1);
        const TooN::Matrix<3,3>& M = so3.get_matrix();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                mat.at<double>(i, j) = M[i][j];
            }
        }

    }

    void Mat_TooNSO3(const cv::Mat& mat, TooN::SO3<> & so3)
    {
        stringstream ss;
        ss << mat.at<double>(0,0) << " "
            << mat.at<double>(0,1) << " " 
            << mat.at<double>(0,2) << " " 
            << mat.at<double>(1,0) << " " 
            << mat.at<double>(1,1) << " " 
            << mat.at<double>(1,2) << " " 
            << mat.at<double>(2,0) << " " 
            << mat.at<double>(2,1) << " " 
            << mat.at<double>(2,2) ;
        ss >> so3;
    }

}
