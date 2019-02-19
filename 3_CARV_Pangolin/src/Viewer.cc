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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

    Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,  ModelDrawer* pModelDrawer,
                   Tracking *pTracking, const string &strSettingPath):
            mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpModelDrawer(pModelDrawer),
            mpTracker(pTracking), mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
    {

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if(fps<1)
            fps=30;
        mT = 1e3/fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if(mImageWidth<1 || mImageHeight<1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];

        mfx = fSettings["Camera.fx"];
        mfy = fSettings["Camera.fy"];
        mcx = fSettings["Camera.cx"];
        mcy = fSettings["Camera.cy"];

        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;

        // Set member matrix
        mProjectionCamera = pangolin::ProjectionMatrix(mImageWidth,mImageHeight,mfx,mfy,mcx,mcy,0.1,1000);
        mViewCamera = pangolin::ModelViewLookAt(0,0,0, 0,0,1, 0.0,-1.0, 0.0);
    }

    void Viewer::Run()
    {
        mbFinished = false;
        mbStopped = false;
        isSaved = false;
        isLoaded = false;
        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",mImageWidth,mImageHeight);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        pangolin::Var<bool> menuSaveCARV("menu.Save CARV",false,true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
        pangolin::Var<bool> menuCameraView("menu.Camera View",true,true);
        pangolin::Var<bool> menuShowModel("menu.Show Model",true,true);
        pangolin::Var<bool> menuShowTexture("menu.Show Texture",false,true);
        pangolin::Var<bool> menuLoadCARV("menu.Load CARV",false,true);
        pangolin::Var<bool> menuReset("menu.Reset",false,false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_map(
                // carv: using calibrated camera center and focal length
                pangolin::ProjectionMatrix(mImageWidth,mImageHeight,mfx,mfy,mcx,mcy,0.1,1000),
                pangolin::ModelViewLookAt(0,0,0, 0,0,1, 0.0,-1.0, 0.0)
        );

        pangolin::Handler3D_VS* pHandler = new pangolin::Handler3D_VS(s_map);

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_map = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -mImageWidth/mImageHeight)
                .SetHandler(pHandler);

        pangolin::OpenGlMatrix MapTwc;
        MapTwc.SetIdentity();

        //cv::namedWindow("ORB-SLAM2: Current Frame");

        bool bFollow = false;
        bool bLocalizationMode = false;

        // carv: camera close up view
        bool bCameraView = true;
        pangolin::OpenGlMatrix projectionAbove = pangolin::ProjectionMatrix(mImageWidth,mImageHeight,mViewpointF,mViewpointF,
                                                                           mImageWidth/2,mImageHeight/2,0.1,1000);
        pangolin::OpenGlMatrix projectionCamera = pangolin::ProjectionMatrix(mImageWidth,mImageHeight,mfx,mfy,mcx,mcy,0.1,1000);
        pangolin::OpenGlMatrix viewAbove = pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0);
        pangolin::OpenGlMatrix viewCamera = pangolin::ModelViewLookAt(0,0,0, 0,0,1, 0.0,-1.0, 0.0);

        while(1)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(MapTwc); //get current view point


            //
            // if(menuFollowCamera && bFollow)
            // {
            //     s_map.Follow(MapTwc);
            // }
            // else if(menuFollowCamera && !bFollow)
            // {
            //     s_map.Follow(MapTwc);
            //     bFollow = true;
            // }
            // else if(!menuFollowCamera && bFollow)
            // {
            //     bFollow = false;
            // }
            //
            // if(menuLocalizationMode && !bLocalizationMode)
            // {
            //     mpSystem->ActivateLocalizationMode();
            //     bLocalizationMode = true;
            // }
            // else if(!menuLocalizationMode && bLocalizationMode)
            // {
            //     mpSystem->DeactivateLocalizationMode();
            //     bLocalizationMode = false;
            // }
            //
            // carv: setup viewpoint to see model
            if(menuCameraView && !bCameraView)
            {
                s_map.SetProjectionMatrix(projectionCamera);
                s_map.SetModelViewMatrix(viewCamera);
                bCameraView = true;
            }
            else if(!menuCameraView && bCameraView)
            {
                s_map.SetProjectionMatrix(projectionAbove);
                s_map.SetModelViewMatrix(viewAbove);
                bCameraView = false;
            }

            d_map.Activate(s_map);



glClearColor(1.0f,1.0f,1.0f,1.0f);
            //mpMapDrawer->DrawMapPoints();
            mpModelDrawer->DrawModelPoints();
            mpModelDrawer->DrawTriangles(MapTwc);
            //mpMapDrawer->DrawCurrentCamera(MapTwc);////////////////////////////////////////////

            // if(menuShowKeyFrames || menuShowGraph)
            //     mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
            // if(menuShowPoints) {
            //     //mpMapDrawer->DrawMapPoints();
            //     // carv: show model points
            //     mpModelDrawer->DrawModelPoints();
            // }
            //
            // CheckGlDieOnError()
            // // carv: show model or triangle with light from camera
            // if(menuShowModel && menuShowTexture) {
            //     mpModelDrawer->DrawModel(mbRGB);
            // }
            // else if (menuShowModel && !menuShowTexture) {
            //     mpModelDrawer->DrawTriangles(MapTwc);
            // }
            // else if (!menuShowModel && menuShowTexture) {
            //     mpModelDrawer->DrawFrame(mbRGB);
            // }

            // Draw ctrl-clicked point
            mpModelDrawer->DrawTarget(MapTwc, pHandler->target);
            CheckGlDieOnError()

            pangolin::FinishFrame();

            // cv::Mat im = mpFrameDrawer->DrawFrame();
            // cv::resize(im, im, cv::Size(), 0.2, 0.2);
            // cv::imshow("ORB-SLAM2: Current Frame",im);
            // cv::waitKey(mT);

            if (menuSaveCARV&&!isSaved)  //save CARV to file
            {
              mpSystem->mpModeler->writeToFile("chris_CARV_Files_save");
              menuSaveCARV = false;
              //isSaved = true;
              //break;
            }

            if (menuLoadCARV&&!isLoaded)  //save CARV to file
            {
              mpSystem->mpModeler->loadModelFromFile("chris_CARV_Files");
              menuLoadCARV = false;
            //  isLoaded = true;
              //break;
            }

            if(menuReset)
            {
                menuShowGraph = false;
                //menuShowKeyFrames = false;
                menuShowPoints = true;
                //menuLocalizationMode = false;
                if(bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                // carv: reset to default
                menuCameraView = true;
                menuShowModel = true;
                menuShowTexture = false;

                mpSystem->Reset();
                menuReset = false;
            }

            if(Stop())
            {
                while(isStopped())
                {
                    usleep(3000);
                }
            }

            if(CheckFinish())
                break;
        }

        SetFinish();
    }

    void Viewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if(!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if(mbFinishRequested)
            return false;
        else if(mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;

    }

    void Viewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }



    cv::Mat Viewer::GetTarget()
    {
        return mpModelDrawer->GetTarget();
    }

    void Viewer::SetTarget(cv::Mat cameraPose, int x, int y)
    {
        pangolin::OpenGlMatrix MapTwc;
        MapTwc.SetIdentity();
        GetOpenGLCameraMatrix(MapTwc, cameraPose);

        GLdouble pc[3];
        GLdouble pw[3];

        const int zl = (HWIN*2+1);
        const int zsize = zl*zl;
        GLfloat zs[zsize];

        glReadBuffer(GL_FRONT);
        glReadPixels(x-HWIN,y-HWIN,zl,zl,GL_DEPTH_COMPONENT,GL_FLOAT,zs);

        GLfloat mindepth = *(std::min_element(zs,zs+zsize));

        const GLint viewport[4] = {0,0,(int)mImageWidth,(int)mImageHeight};
        const pangolin::OpenGlMatrix proj = mProjectionCamera;
        pangolin::glUnProject(x, y, mindepth, pangolin::Identity4d, proj.m, viewport, &pc[0], &pc[1], &pc[2]);

        const pangolin::OpenGlMatrix mv = mViewCamera;

        GLdouble T_wc[3*4];
        pangolin::LieSE3from4x4(T_wc, mv.Inverse().m );
        pangolin::LieApplySE3vec(pw, T_wc, pc);

        Eigen::Vector3d target = Eigen::Map<const Eigen::Matrix<GLdouble,3,1>>(pw).cast<double>();

        // Send to model drawer
        mpModelDrawer->SetTarget(MapTwc, target);

    }

    void Viewer::GetOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, cv::Mat cameraPose)
    {
        if(!cameraPose.empty())
        {
            cv::Mat Rwc(3,3,CV_32F);
            cv::Mat twc(3,1,CV_32F);
            Rwc = cameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*cameraPose.rowRange(0,3).col(3);

            M.m[0] = Rwc.at<float>(0,0);
            M.m[1] = Rwc.at<float>(1,0);
            M.m[2] = Rwc.at<float>(2,0);
            M.m[3]  = 0.0;

            M.m[4] = Rwc.at<float>(0,1);
            M.m[5] = Rwc.at<float>(1,1);
            M.m[6] = Rwc.at<float>(2,1);
            M.m[7]  = 0.0;

            M.m[8] = Rwc.at<float>(0,2);
            M.m[9] = Rwc.at<float>(1,2);
            M.m[10] = Rwc.at<float>(2,2);
            M.m[11]  = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15]  = 1.0;
        }
        else
            M.SetIdentity();
    }

}


// Custom mouse handler
namespace pangolin {

    Handler3D_VS::Handler3D_VS(OpenGlRenderState& cam_state, AxisDirection enforce_up, float trans_scale, float zoom_fraction)
            : Handler3D(cam_state, enforce_up, trans_scale, zoom_fraction)
    {
        target.setZero();
    }

    void Handler3D_VS::Mouse(View &display, MouseButton button, int x, int y, bool pressed, int button_state)
    {
        Handler3D::Mouse(display, button, x, y, pressed, button_state);

        if (button_state & MouseButtonLeft && button_state & KeyModifierCtrl) {
            GLprecision pc[3];
            GLprecision pw[3];

            // int mImageHeight = 720;
            // const int zl = 1;//(HWIN*2+1);
            // const int zsize = zl*zl;
            // GLfloat zs[zsize];
            // cout<<"windows height"<<mImageHeight<<endl;
            //
            // glReadBuffer(GL_FRONT);
            // glReadPixels(x,mImageHeight-y-1,zl,zl,GL_DEPTH_COMPONENT,GL_FLOAT,zs);
            // //cout<<"glReadPixels x: "<<x<<" y:"<<mImageHeight-y-1<<endl;
            // cout<<"glut coordinate x: "<<x<<" y:"<<mImageHeight-y-1<<endl;
            // GLfloat mindepth = *(std::min_element(zs,zs+zsize));
            // if(mindepth == 1) mindepth = (GLfloat)last_z;
            // cout<<"mindepth "<<mindepth<<endl;
            // PixelUnproject(display, x, mImageHeight-y-1, mindepth, pc);
            // cout<<"3D coordinates:"<<endl;
            // cout<<"x "<<pc[0]<<"  y :"<<pc[1]<<"  z: "<<pc[2]<<endl;
            // const pangolin::OpenGlMatrix mv = cam_state->GetModelViewMatrix();

            const int zl = (HWIN*2+1);
            const int zsize = zl*zl;
            GLfloat zs[zsize];

            glReadBuffer(GL_FRONT);
            glReadPixels(x-HWIN, y - HWIN,zl,zl,GL_DEPTH_COMPONENT,GL_FLOAT,zs);
            cout<<"openGL coordinate x: "<<x<<" y:"<<y<<endl;
            GLfloat mindepth = *(std::min_element(zs,zs+zsize));
            if(mindepth == 1) mindepth = (GLfloat)last_z;

            PixelUnproject(display, x, y, mindepth, pc);

            const pangolin::OpenGlMatrix mv = cam_state->GetModelViewMatrix();

            GLprecision T_wc[3*4];
            LieSE3from4x4(T_wc, mv.Inverse().m );
            LieApplySE3vec(pw, T_wc, pc);

            target = Eigen::Map<const Eigen::Matrix<GLprecision,3,1>>(pw).cast<double>();
        }
    }

}
