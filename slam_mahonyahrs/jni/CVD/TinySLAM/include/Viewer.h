#include <stdio.h>
#include <stdlib.h>

#include <iostream>

#include <pangolin/pangolin.h>

#include <mutex>

#include "Mapping.h"
#include "VisionTracker.h"

class Viewer
{
    public:
        Viewer() = default;
        Viewer(Mapping* _map, VisionTracker* _tracker):map(_map), 
                tracker(_tracker), isFinished(false) {};

        void run();
        void requestFinish();
        bool checkFinished();
        void drawKeyFrames();
        void drawMapPoints();
        void drawCameraNow();
        void drawMapAxis();

    private:
        Mapping* map;
        VisionTracker* tracker;

        std::mutex mMutexFinish;
        bool isFinished;

};
