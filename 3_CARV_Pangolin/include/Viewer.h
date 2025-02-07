/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

// carv include
#include "Modeler/ModelDrawer.h"

#include <mutex>


// half of patch size (getting pixel depth)
//#define HWIN 8
#define HWIN 1

namespace ORB_SLAM2
{
    class Tracking;
    class FrameDrawer;
    class MapDrawer;
    class System;

    // carv class declaration
    class ModelDrawer;

    class Viewer
    {
    public:
        // carv initialize with modeldrawer
        Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, ModelDrawer* pModelDrawer,
               Tracking *pTracking, const string &strSettingPath);

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        void Release();
        bool isSaved;//CARV
        bool isLoaded;//CARV
        // set target by camera pose and image coordinates
        void SetTarget(cv::Mat cameraPose, int x, int y);
        cv::Mat GetTarget();
        void GetOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, cv::Mat cameraPose);

        pangolin::OpenGlMatrix mProjectionCamera;
        pangolin::OpenGlMatrix mViewCamera;

    private:

        bool Stop();

        System* mpSystem;
        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;

        // carv modeldrawer pointer
        ModelDrawer* mpModelDrawer;

        Tracking* mpTracker;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
        float mfx, mfy, mcx, mcy;
        bool mbRGB;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;

    };

}

// Custom mouse handler
namespace pangolin {
    struct Handler3D_VS : Handler3D {
        Handler3D_VS(OpenGlRenderState& cam_state, AxisDirection enforce_up=AxisNone, float trans_scale=0.01f, float zoom_fraction= PANGO_DFLT_HANDLER3D_ZF);
        void Mouse(View &display, MouseButton button, int x, int y, bool pressed, int button_state);
        Eigen::Vector3d target;
    };
}

#endif // VIEWER_H
