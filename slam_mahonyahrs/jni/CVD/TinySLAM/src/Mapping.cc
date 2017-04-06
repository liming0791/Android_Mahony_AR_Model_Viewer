#include "Mapping.h"

#include "RedirectPrintf.h"

void Mapping::InitMap(ImageFrame& lf, ImageFrame& rf)
{

    cv::Mat T1, T2, pts_4d;
    cv::hconcat(lf.R.t(), -lf.t, T1);
    cv::hconcat(rf.R.t(), -rf.t, T2);

    std::vector< cv::Point2f > pts_1, pts_2;
    std::vector< int > pt_idx;
    pt_idx.reserve(lf.undisPoints.size());
    pts_2.reserve(lf.undisPoints.size());
    pts_2.reserve(lf.undisPoints.size());

    for (int i = 0, _end = (int)lf.undisPoints.size(); i < _end; i++) {
        if ( rf.undisTrackedPoints[i].x > 0 ) {
            pt_idx.push_back(i);
            pts_1.push_back(K->pixel2device(
                        lf.undisPoints[i].x,
                        lf.undisPoints[i].y));
            pts_2.push_back(K->pixel2device(
                        rf.undisTrackedPoints[i].x,
                        rf.undisTrackedPoints[i].y));
        }
    }

    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    cout << "Triangulate map point:" << endl;

    for (int i = 0; i < pts_4d.cols; i++) {
        float w = pts_4d.at<float>(3, i);
        cv::Point3f* mpt = new cv::Point3f(
                pts_4d.at<float>(0, i)/w,
                pts_4d.at<float>(1, i)/w,
                pts_4d.at<float>(2, i)/w);

        cout << *mpt << endl;

        mapPoints.insert(mpt);     // Insert map point pointer to std::set
        lf.map_2d_3d.insert(  // Insert bimap key-val to boost::bimap in lf
                Map_2d_3d_key_val(pt_idx[i], mpt));     
    }

    keyFrames.push_back(&lf);

}

void Mapping::ClearMap()
{
    for (set< cv::Point3f* >::iterator iter = mapPoints.begin(),
            i_end = mapPoints.end(); iter != i_end; iter++) {
        cv::Point3f * p = *iter;
        if (p != NULL)
            delete(p);
    }
    
    mapPoints.clear();
    keyFrames.resize(0);
}
