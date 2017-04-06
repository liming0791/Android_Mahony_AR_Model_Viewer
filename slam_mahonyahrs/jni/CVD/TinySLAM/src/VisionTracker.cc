#include "VisionTracker.h"
#include "MedianFilter.h"
#include "Converter.h"
#include "Timer.h"

#include "RedirectPrintf.h"

bool VisionTracker::TrackMonocular(ImageFrame& f)
{
    if (state == NOTINITIALIZED) {
        f.extractFAST();
        refFrame = f;
        state = INITIALIZING;
        return false;
    } else if (state == INITIALIZING) {
        //TryInitialize(f);
        TryInitializeByG2O(f);
        std::cout << "TryInitialize done." << std::endl;
        return false;
    } else {
        f.opticalFlowFAST(refFrame);
        return TrackPose3D2D(refFrame, f);
    }

}

void VisionTracker::TryInitialize(ImageFrame& f)
{

    // Track FAST Points
    f.opticalFlowFAST(refFrame);

    // Track Pose 2D-2D
    std::vector< cv::Point2f > lp, rp;
    lp.reserve(f.undisTrackedPoints.size());
    rp.reserve(f.undisTrackedPoints.size());

    for (int i = 0, _end = (int)f.undisTrackedPoints.size(); 
            i < _end; i++ ) {
        if (f.undisTrackedPoints[i].x > 0) {
            lp.push_back(refFrame.undisPoints[i]);
            rp.push_back(f.undisTrackedPoints[i]);
        }
    }

    double ratio_tracked = (double)lp.size() / 
            (double)f.undisTrackedPoints.size();
    if (ratio_tracked < 0.3) {
        printf("Initialize Tracked points num too small!"
                " less than 0.3\n");
        return;
    }

    double disparty = 0;
    for (int i = 0, _end = (int)lp.size(); i < _end; i++) {
        disparty = disparty + (lp[i].x - rp[i].x)*(lp[i].x - rp[i].x)
                + (lp[i].y - rp[i].y)*(lp[i].y - rp[i].y);
    }

    if ( disparty < 225.0 * (int)lp.size() ) {
        printf("Initialize disparty too small, less than 15 average!\n");
        return;
    }

    cv::Point2d principal_point(K->cx, K->cy);
    double focal_length = (K->fx + K->fy)/2;
    cv::Mat inliners;
    cv::Mat essential_matrix = cv::findEssentialMat(lp, rp, 
            focal_length, principal_point, cv::RANSAC, 0.999, 1.0, inliners);

    int num_inliners = 0;
    for (int i = 0, _end = (int)lp.size(); i < _end; i++) {
        if (inliners.at<unsigned char>(i) == 1)
            ++num_inliners;
    }

    double ratio_inliners = (double)num_inliners / (int)lp.size();

    if (ratio_inliners < 0.9) {
       printf("Initialize essential matrix inliners num too small!"
               " less than 0.9\n"); 
       return;
    }

    cout << "essential_matrix: " << endl
        << essential_matrix << endl;

    cv::Mat R, t;
    cv::recoverPose(essential_matrix, lp, rp, R, t, focal_length, principal_point);

    double ratio_shift = t.at<double>(0)*t.at<double>(0)
            + t.at<double>(1)*t.at<double>(1)
            + t.at<double>(2)*t.at<double>(2);

    if (ratio_shift < 0.0025) {
        printf("Initialize recover pose shift too small!"
                " Is %f\n", ratio_shift);
        return ;
    }


    cout << "R: " << endl
        << R << endl;
    cout << "t: " << endl
        << t << endl;

    printf("Shift: %f\n", ratio_shift);

    mR = f.R = R.t();
    mt = f.t = -t;

    TooN::SO3<> Rot;
    TooN::Vector<3> Trans;
    Converter::Mat_TooNSO3(mR, Rot);
    Trans[0] = mt.at<double>(0);
    Trans[1] = mt.at<double>(1);
    Trans[2] = mt.at<double>(2);
    cout << "Rot: " << endl
        << Rot <<endl;
    cout << "Trans: " << endl
        << Trans << endl;

    mTcwNow = f.mTcw = refFrame.mTcw * TooN::SE3<>(Rot, Trans);
    cout << "mTcw: " << endl
        << f.mTcw << endl;

    // Init Mapping
    map->InitMap(refFrame, f);

    // Set state
    state = INITIALIZED;

}

void VisionTracker::TryInitializeByG2O(ImageFrame& f)
{

    // Track FAST Points
    f.opticalFlowFAST(refFrame);

    // Track Pose 2D-2D
    std::vector< cv::Point2f > lp, rp;
    vector< int > pt_idx;
    lp.reserve(f.undisTrackedPoints.size());
    rp.reserve(f.undisTrackedPoints.size());
    pt_idx.reserve(f.undisTrackedPoints.size());

    for (int i = 0, _end = (int)f.undisTrackedPoints.size();
            i < _end; i++ ) {
        if (f.undisTrackedPoints[i].x > 0) {
            lp.push_back(refFrame.undisPoints[i]);
            rp.push_back(f.undisTrackedPoints[i]);
            pt_idx.push_back(i);
        }
    }

    // check tracked percent
    double ratio_tracked = (double)lp.size() /
            (double)f.undisTrackedPoints.size();
    if (ratio_tracked < 0.3) {
        printf("Initialize Tracked points num too small!"
                " less than 0.3\n");
        return;
    }

    // check disparty
    double disparty = 0;
    for (int i = 0, _end = (int)lp.size(); i < _end; i++) {
        disparty = disparty + (lp[i].x - rp[i].x)*(lp[i].x - rp[i].x)
                + (lp[i].y - rp[i].y)*(lp[i].y - rp[i].y);
    }
    if ( disparty < 225.0 * (int)lp.size() ) {
        printf("Initialize disparty too small, less than 15 average!\n");
        return;
    }

    // init by g2o
    cout << "Set optimizer." << endl;

    // Optimizer
    g2o::SparseOptimizer optimizer;

    // linear solver
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse< g2o::BlockSolver_6_3::PoseMatrixType >();

    // block solver
    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );

    // optimization algorithm
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );

    optimizer.setAlgorithm( algorithm );
    optimizer.setVerbose( false );

    cout << "add pose vertices." << endl;
    // add pose vertices
    g2o::VertexSE3Expmap* v1 = new g2o::VertexSE3Expmap();
    v1->setId(0);
    v1->setFixed(true);
    v1->setEstimate(g2o::SE3Quat());
    optimizer.addVertex(v1);

    g2o::VertexSE3Expmap* v2 = new g2o::VertexSE3Expmap();
    v2->setId(1);
    v2->setEstimate(g2o::SE3Quat());
    optimizer.addVertex(v2);

    cout << "add 3d points vertices" << endl;
    // add 3d point vertices
    for (int i = 0, _end = (int)lp.size(); i < _end; i++) {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId(2+i);
        double z = 10;
        double x = ( lp[i].x - K->cx ) / K->fx * z;
        double y = ( lp[i].y - K->cy ) / K->fy * z;
        v->setMarginalized(true);
        v->setEstimate( Eigen::Vector3d(x, y, z) );
        optimizer.addVertex( v );
    }

    cout << "add camera parameters" << endl;
    // prepare camera parameters
    g2o::CameraParameters* camera = new g2o::CameraParameters( (K->fx + K->fy)/2, Eigen::Vector2d(K->cx, K->cy), 0 );
    camera->setId(0);
    optimizer.addParameter(camera);

    cout << "add edges" << endl;
    // prepare edges
    vector< g2o::EdgeProjectXYZ2UV* > edges;
    for (int i = 0, _end = (int)lp.size(); i < _end; i++) {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast< g2o::VertexSBAPointXYZ* > (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast< g2o::VertexSE3Expmap* > (optimizer.vertex(0)) );

        edge->setMeasurement( Eigen::Vector2d(lp[i].x, lp[i].y) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);

        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back( edge );
    }

    for (int i = 0, _end = (int)rp.size(); i < _end; i++) {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast< g2o::VertexSBAPointXYZ* > (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast< g2o::VertexSE3Expmap* > (optimizer.vertex(1)) );

        edge->setMeasurement( Eigen::Vector2d(rp[i].x, rp[i].y) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);

        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back( edge );
    }

    cout << "optimization" << endl;
    // optimization
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // num of inliers
    vector< int > inliers(edges.size()/2, 1);
    int num_vert = (int)inliers.size();
    for ( int i = 0, _end = (int)edges.size(); i < _end; i++) {
        g2o::EdgeProjectXYZ2UV* e = edges[i];
        e->computeError();
        if (e->chi2() > 1) {
            inliers[i%num_vert] = 0;
            cout << "error = " << e->chi2() << endl;
        }
    }
    int num_inliers = 0;
    for ( int i = 0, _end = (int)inliers.size(); i < _end; i++) {
        num_inliers += inliers[i];
    }
    cout << "num inliers: " << num_inliers << endl;

    // check inliers
    double ratio_inlier = (double)num_inliers / (double)lp.size();
    if (ratio_inlier < 0.8) {
        printf("Inliers too small, less than 0.8 !\n");
        return;
    }

    // SE3 estimate
    g2o::VertexSE3Expmap* v =
            dynamic_cast< g2o::VertexSE3Expmap* >( optimizer.vertex(1) );
    Eigen::Isometry3d pose = v->estimate();
    // set optimization pose result, do not forget this step
    Eigen::Matrix4d T = pose.matrix();
    cv::Mat res;
    cv::eigen2cv(T, res);
    cv::Mat R = res.rowRange(0,3).colRange(0,3);
    cv::Mat t = res.rowRange(0,3).col(3);
    mR = f.R = R.t();
    mt = f.t = -t;

    cout << "mR: " << endl;
    cout << mR <<  endl;

    cout << "mt: " << endl;
    cout << mt << endl;

    // points estimate
    for (int i = 0, _end = (int)lp.size(); i < _end; i++ ) {
        if (inliers[i] == 0)
            continue;
        g2o::VertexSBAPointXYZ* v =
                dynamic_cast< g2o::VertexSBAPointXYZ* >
                ( optimizer.vertex(i+2) );
        Eigen::Vector3d pos = v->estimate();
        // set Mapping points and lf points
        cv::Point3f* mpt = new cv::Point3f(pos[0], pos[1], pos[2]);
        map->mapPoints.insert(mpt);
        refFrame.map_2d_3d.insert(
                Map_2d_3d_key_val(pt_idx[i], mpt));
    }

    // add keyFrame
    map->keyFrames.push_back(&refFrame);

    // Set state
    state = INITIALIZED;

}

void VisionTracker::reset()
{
    map->ClearMap();
    state = NOTINITIALIZED;
}

void VisionTracker::TrackPose2D2D(const ImageFrame& lf, ImageFrame& rf)
{
    std::vector< cv::Point2f > lp, rp;
    lp.reserve(rf.undisTrackedPoints.size());
    rp.reserve(rf.undisTrackedPoints.size());

    for (int i = 0, _end = (int)rf.undisTrackedPoints.size(); i < _end; i++ ) {
        if (rf.undisTrackedPoints[i].x > 0) {
            lp.push_back(lf.undisPoints[i]);
            rp.push_back(rf.undisTrackedPoints[i]);
        }
    }

    cv::Point2d principal_point(K->cx, K->cy);
    double focal_length = (K->fx + K->fy)/2;
    cv::Mat essential_matrix = cv::findEssentialMat(lp, rp, focal_length, principal_point);
    cout << "essential_matrix: " << endl
        << essential_matrix << endl;

    cv::Mat R, t;
    cv::recoverPose(essential_matrix, lp, rp, R, t, focal_length, principal_point);
    cout << "R: " << endl
        << R << endl;
    cout << "t: " << endl
        << t << endl;

    mR = rf.R = R.t();
    mt = rf.t = -t;

    TooN::SO3<> Rot;
    TooN::Vector<3> Trans;
    Converter::Mat_TooNSO3(mR, Rot);
    Trans[0] = mt.at<double>(0);
    Trans[1] = mt.at<double>(1);
    Trans[2] = mt.at<double>(2);
    cout << "Rot: " << endl
        << Rot <<endl;
    cout << "Trans: " << endl
        << Trans << endl;

    mTcwNow = rf.mTcw = lf.mTcw * TooN::SE3<>(Rot, Trans);
    cout << "mTcw: " << endl
        << rf.mTcw << endl;
}

bool VisionTracker::TrackPose3D2D(const ImageFrame& lf, ImageFrame& rf)
{
    cv::Mat KMat = 
            (cv::Mat_<double> (3,3) << K->fx, 0, K->cx, 0, K->fy, K->cy, 0, 0, 1);
    vector< cv::Point2f > pts_2d;
    vector< cv::Point3f > pts_3d;
    pts_2d.reserve(lf.undisPoints.size());
    pts_3d.reserve(lf.undisPoints.size());

    double num_mappoint = 0;
    for (Map_2d_3d::const_iterator iter = lf.map_2d_3d.begin(), 
            i_end = lf.map_2d_3d.end(); iter != i_end; iter++) {
        num_mappoint++;
        if (rf.undisTrackedPoints[iter->left].x > 0) {
            pts_2d.push_back(rf.undisTrackedPoints[iter->left]);    
            pts_3d.push_back(*(iter->right));
        }
    }

    if ( (int)pts_2d.size() / num_mappoint  < 0.3) {
        printf("Tracked points less than ratio 0.3, can not track pose 3d-2d!\n");
        return false;
    }

    //cv::Mat r, t, R, inliers;
    //cv::solvePnPRansac (pts_3d, pts_2d, KMat, cv::Mat(), r, t, false, 100, 8.0, 0.99, inliers);   // opencv solvePnP result is bad, 
    //cv::Rodrigues(r, R);                                                                          // do not use it

    cv::Mat R = lf.R.t();                                       // set initial pose as the refImage
    cv::Mat t = -lf.t;
    bundleAdjustment3D2D(pts_3d, pts_2d, KMat, R, t);           // optimize the pose by g2o

    //cout << "BA:" << endl;
    //cout << R << endl;
    //cout << t << endl;
    
    // use Median filter to make the result stable
    TooN::SO3<> so3;
    Converter::Mat_TooNSO3(R, so3);
    TooN::Vector<3> w = so3.ln(); 
    w[0] = medianFilter[0].filterAdd(w[0]);
    w[1] = medianFilter[1].filterAdd(w[1]);
    w[2] = medianFilter[2].filterAdd(w[2]);
    t.at<double>(0) = medianFilter[3].filterAdd(t.at<double>(0));
    t.at<double>(1) = medianFilter[4].filterAdd(t.at<double>(1));
    t.at<double>(2) = medianFilter[5].filterAdd(t.at<double>(2));
    so3 = TooN::SO3<>::exp(w);
    Converter::TooNSO3_Mat(so3, R);

    mR = R.t();
    mt = -t;

    return true;
}

void VisionTracker::bundleAdjustment3D2D(
        const vector< cv::Point3f > & points_3d,
        const vector< cv::Point2f > & points_2d,
        const cv::Mat & KMat,
        cv::Mat& R, cv::Mat& t)
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // Set Frame vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    cv::cv2eigen(R, R_mat);
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
                R_mat,
                Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                ) );
    optimizer.addVertex ( pose );

    // Set MapPoint vertices
    int index = 1;
    for ( const cv::Point3f p:points_3d )   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
            KMat.at<double> ( 0,0 ), Eigen::Vector2d ( KMat.at<double> ( 0,2 ), KMat.at<double> ( 1,2 ) ), 0
            );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // Set 2d Point edges
    index = 1;
    for ( const cv::Point2f p:points_2d )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    TIME_BEGIN()
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    TIME_END("g2o optimization")

    //cout<<endl<<"after optimization:"<<endl;
    //cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;

    // set optimization result back, do not forget this step
    Eigen::Matrix4d T = Eigen::Isometry3d ( pose->estimate() ).matrix();
    cv::Mat res;
    cv::eigen2cv(T, res);
    R = res.rowRange(0,3).colRange(0,3);
    t = res.rowRange(0,3).col(3);

}

cv::Mat VisionTracker::GetTcwMatNow()
{

    if (mR.empty()||mt.empty())
        return cv::Mat();

    cv::Mat res = cv::Mat::eye(4, 4, CV_64FC1);
    mR.copyTo(res.rowRange(0,3).colRange(0,3));
    mt.copyTo(res.rowRange(0,3).col(3));
    return res;
}

cv::Mat VisionTracker::GetTwcMatNow()
{

    if (mR.empty()||mt.empty())
        return cv::Mat();

    cv::Mat res = cv::Mat::eye(4, 4, CV_64FC1);
    cv::Mat Rt = mR.t();
    cv::Mat _t = -mt;
    Rt.copyTo(res.rowRange(0,3).colRange(0,3));
    _t.copyTo(res.rowRange(0,3).col(3));
    return res;
}
