#include "ImageFrame.h"
#include "Timer.h"

#include "RedirectPrintf.h"

ImageFrame::ImageFrame(const cv::Mat& frame, CameraIntrinsic* _K): //mSBI(CVD::ImageRef(32, 24)),
        R(cv::Mat::eye(3,3, CV_64FC1)), t(cv::Mat::zeros(3,1, CV_64FC1)), K(_K)
{
    if (frame.channels()==3) {
        cv::cvtColor(frame, image, CV_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, image, CV_BGRA2GRAY);
    } else {
        frame.copyTo(image);
    }

    //cv::Mat s_img;
    //cv::resize(image, s_img, cv::Size(32, 24));
    //cv::blur(s_img, s_img, cv::Size(3,3));

    //memcpy(mSBI.begin(), s_img.data, 32*24*sizeof(unsigned char));

    //mSBI.resize(CVD::ImageRef(32, 24));

}

ImageFrame::ImageFrame(const ImageFrame& imgFrame): 
    image(imgFrame.image), //mSBI(imgFrame.mSBI),
    keyPoints(imgFrame.keyPoints),
    mTcw(imgFrame.mTcw), R(imgFrame.R), t(imgFrame.t), mRefFrame(imgFrame.mRefFrame), K(imgFrame.K)
{
    
}

void ImageFrame::extractFAST()
{
    int thres = 40;
    int low = 400;
    int high = 500;
    TIME_BEGIN()
    cv::FAST(image, keyPoints, thres);

    while((int)keyPoints.size() < low || (int)keyPoints.size() > high)
    {
       if ((int)keyPoints.size() < low) {
            thres -= 2;
            cv::FAST(image, keyPoints, thres);
       } else if ((int)keyPoints.size() > high) {
            thres += 2;
            cv::FAST(image, keyPoints, thres);
       }
    }
    TIME_END("Extract FAST")

    points.reserve(keyPoints.size());       // use reserve to improve performance
    points.resize(0);
    undisPoints.reserve(keyPoints.size());
    undisPoints.resize(0);
    for (int i = 0, _end = (int)keyPoints.size(); i < _end; i++) {
        points.push_back( keyPoints[i].pt );
        undisPoints.push_back( K->undistort(points[i].x, points[i].y) );
    }

    trackedPoints = points;
    undisTrackedPoints = undisPoints;

    std::cout << "Extract FAST: " << points.size() << std::endl;
    
}

void ImageFrame::opticalFlowFAST(ImageFrame& refFrame)
{
    mRefFrame = &refFrame;

    // optical flow
    cv::Mat status, err;
    TIME_BEGIN()
    cv::calcOpticalFlowPyrLK(refFrame.image, image, 
            refFrame.points, trackedPoints, status, err) ;
    TIME_END("Optical Flow")

    // homography estimation validation 
    undisTrackedPoints.reserve(trackedPoints.size());
    undisTrackedPoints.resize(0);
    for (int i = 0, _end = (int)trackedPoints.size(); i < _end; i++) {
        undisTrackedPoints.push_back( 
                K->undistort(trackedPoints[i].x, trackedPoints[i].y) );
    }

    cv::Mat inliner;
    TIME_BEGIN()
    cv::findHomography(refFrame.undisPoints, undisTrackedPoints, 
            cv::RANSAC, 3, inliner);
    TIME_END("Homography estimation")

    for (int i = 0, _end = (int)trackedPoints.size(); i < _end; i++) {
        if (inliner.at<unsigned char>(i) == 1) {

        } else {
            trackedPoints[i].x = trackedPoints[i].y = 0;
            undisTrackedPoints[i].x = undisTrackedPoints[i].y = 0;
        }
    }

}

void ImageFrame::opticalFlowTrackedFAST(ImageFrame& lastFrame)
{
    mRefFrame = &lastFrame;

    vector< cv::Point2f > pts, undis_pts, undis_flow_pts,flow_pts;
    vector< int > idxs;
    pts.reserve(lastFrame.trackedPoints.size());
    undis_pts.reserve(lastFrame.trackedPoints.size());
    flow_pts.reserve(lastFrame.trackedPoints.size());
    undis_flow_pts.reserve(lastFrame.trackedPoints.size());
    idxs.reserve(lastFrame.trackedPoints.size());

    for (int i = 0, _end = (int)lastFrame.trackedPoints.size(); 
            i < _end; i++) {
        if (lastFrame.trackedPoints[i].x > 0) {
            pts.push_back(lastFrame.trackedPoints[i]);
            undis_pts.push_back(lastFrame.undisTrackedPoints[i]);
            idxs.push_back(i);
        }
    }

    cv::Mat status, err;
    TIME_BEGIN()
        cv::calcOpticalFlowPyrLK(lastFrame.image, image, 
                pts, flow_pts, status, err) ;
    TIME_END("Optical Flow")

    for (int i = 0, _end = (int)flow_pts.size(); i < _end; i++) {
        undis_flow_pts.push_back( 
                K->undistort(flow_pts[i].x, flow_pts[i].y) );
    }

    cv::Mat inliner;
    TIME_BEGIN()
    cv::findHomography(undis_pts, undis_flow_pts, 
            cv::RANSAC, 3, inliner);
    TIME_END("Homography estimation")

    trackedPoints.resize(lastFrame.trackedPoints.size());
    undisTrackedPoints.resize(lastFrame.trackedPoints.size());
    fill(trackedPoints.begin(), 
            trackedPoints.end(), cv::Point2f(-1,-1));
    fill(undisTrackedPoints.begin(), 
            undisTrackedPoints.end(), cv::Point2f(-1,-1));
    for (int i = 0, _end = (int)undis_flow_pts.size(); i < _end; i++) {
        if (inliner.at<unsigned char>(i) == 1) {
          trackedPoints[idxs[i]] = flow_pts[i];
          undisTrackedPoints[idxs[i]] = undis_flow_pts[i];
        } else {

        }
    }

}

//void ImageFrame::SBITrackFAST(ImageFrame& refFrame)
//{
//    mRefFrame = &refFrame;
//
//    CVD::Homography<8> homography;
//    CVD::StaticAppearance appearance;
//    CVD::Image< TooN::Vector<2> > greImg
//            = CVD::Internal::gradient<TooN::Vector<2>, unsigned char>(refFrame.mSBI);
//    CVD::Internal::esm_opt(homography, appearance, refFrame.mSBI, greImg, mSBI, 40, 1e-8, 1.0);
//    TooN::Matrix<3> H = homography.get_matrix();


//    H(0,2) = H(0,2) * 20.f;
//    H(1,2) = H(1,2) * 20.f;
//    H(2,0) = H(2,0) / 20.f;
//    H(2,1) = H(2,1) / 20.f;

//    keyPoints.resize(0);

//    for (int i = 0, _end = (int)refFrame.keyPoints.size(); i < _end; i++ ) {
//        TooN::Vector<3> P;
//        P[0] = refFrame.keyPoints[i].pt.x;
//       P[1] = refFrame.keyPoints[i].pt.y;
//        P[2] = 1;
//        TooN::Vector<3> n_P = H * P;
//        keyPoints.push_back(cv::KeyPoint(n_P[0]/n_P[2], n_P[1]/n_P[2], 10));
//    }

//}

cv::Mat ImageFrame::GetTcwMat()
{
    if (R.empty() || t.empty())
        return cv::Mat();

    cv::Mat res = cv::Mat::eye(4, 4, CV_64FC1);
    R.copyTo(res.rowRange(0,3).colRange(0,3));
    t.copyTo(res.rowRange(0,3).col(3));
    return res;
}

cv::Mat ImageFrame::GetTwcMat()
{
    if (R.empty() || t.empty() )
        return cv::Mat();

    cv::Mat res = cv::Mat::eye(4, 4, CV_64FC1);
    cv::Mat Rt = R.t();
    cv::Mat _t = -t;
    Rt.copyTo(res.rowRange(0,3).colRange(0,3));
    _t.copyTo(res.rowRange(0,3).col(3));
    return res;
}
