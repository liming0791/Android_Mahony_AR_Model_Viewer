#include "Viewer.h"

#include "RedirectPrintf.h"

void Viewer::run()
{
    pangolin::CreateWindowAndBind("Viewer", 1024, 768);

    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // TODO::Create menu
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 
            512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0, -1, 0)
            );

    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0, 1, pangolin::Attach::Pix(175), 1.f, -1024.f/768.f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix M;
    M.SetIdentity();

    while(1) {
        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.f, 1.f, 1.f, 1.f);

        drawCameraNow();

        drawKeyFrames();

        drawMapPoints();

        drawMapAxis();

        pangolin::FinishFrame();

        if (checkFinished())
            break;
    }

}

void Viewer::drawCameraNow()
{
    const float w = 1;
    const float h = w*0.75;
    const float z = w*0.6;

    if (!tracker->mR.empty()) {
        cv::Mat Tcw = tracker->GetTcwMatNow().t();

        glPushMatrix();

        glMultMatrixd(Tcw.ptr<double>(0));

        glLineWidth(2);
        glColor3f(0.0f,1.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }

}

void Viewer::drawKeyFrames()
{
    const float w = 1;
    const float h = w*0.75;
    const float z = w*0.6;

    for (int i = 0, _end = (int)map->keyFrames.size(); i < _end; i++) {
        ImageFrame* pKF = map->keyFrames[i];
        cv::Mat Tcw = pKF->GetTcwMat().t();

        glPushMatrix();

        glMultMatrixd(Tcw.ptr<double>(0));

        glLineWidth(2);
        glColor3f(0.0f,0.0f,1.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();

    }

}

void Viewer::drawMapPoints()
{

    glPointSize(3);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for (set< cv::Point3f* >::iterator s_it = map->mapPoints.begin(), 
            s_end = map->mapPoints.end(); s_it != s_end; s_it++) {
        cv::Point3f* pt = (*s_it);
        glVertex3f((*pt).x, (*pt).y, (*pt).z);
    }

    glEnd();

}

void Viewer::drawMapAxis()
{   
    glLineWidth(2);
    
    // Z Axis blue
    glColor3f(0.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0,0,1);
    glEnd();

    // Y Axis red
    glColor3f(1.0f,0.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0,1,0);
    glEnd();

    // X Axis green
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(1,0,0);
    glEnd();
}

void Viewer::requestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    isFinished = true;
}

bool Viewer::checkFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return isFinished;
}
