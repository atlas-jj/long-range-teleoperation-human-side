//
// Created by shida on 06/12/16.
//

#include <Eigen/Geometry>
#include "Modeler/ModelDrawer.h"

namespace ORB_SLAM2
{
    ModelDrawer::ModelDrawer(Map* pMap):mpMap(pMap),mbModelUpdateRequested(false), mbModelUpdateDone(true)
    {
        target.setZero();
    }

    void ModelDrawer::DrawModel(bool bRGB)
    {
        //TODO: find a way to optimize rendering keyframes

        // select 10 KFs
        int numKFs = 1;
        vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

        std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        vector<KeyFrame*> vpKFtex;

        for(auto it = vpKFs.rbegin(); it != vpKFs.rend(); ++it) {
            if ((int)vpKFtex.size() >= numKFs)
                break;
            KeyFrame *kf = *it;
            kf->SetNotEraseDrawer();
            if (kf->isBad()) {
                kf->SetEraseDrawer();
                continue;
            }
            vpKFtex.push_back(kf);
        }


        UpdateModel();

        if ((int)vpKFtex.size() >= numKFs) {

            static unsigned int frameTex = 0;

            if (!frameTex)
                glGenTextures(numKFs, &frameTex);

            cv::Size imSize = vpKFtex[0]->mImage.size();

            cv::Mat mat_array[numKFs];
            for (int i = 0; i < numKFs; i++) {
                if (vpKFtex[i]->mImage.channels() == 3) {
                    mat_array[i] = vpKFtex[i]->mImage;
                } else {
                    //num of channels is not 3, something is wrong
                    for (auto it = vpKFtex.begin(); it != vpKFtex.end(); it++)
                        (*it)->SetEraseDrawer(); //release the keyframes
                    return;
                }
            }


            // cv::Mat texture;
            // cv::vconcat(mat_array, numKFs, texture);
            //
            // glEnable(GL_TEXTURE_2D);
            //
            // glBindTexture(GL_TEXTURE_2D, frameTex);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            // // image are saved in RGB format, grayscale images are converted
            // if (bRGB) {
            //     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
            //                  imSize.width, imSize.height*numKFs, 0,
            //                  GL_BGR,
            //                  GL_UNSIGNED_BYTE,
            //                  texture.data);
            // } else {
            //     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
            //                  imSize.width, imSize.height*numKFs, 0,
            //                  GL_RGB,
            //                  GL_UNSIGNED_BYTE,
            //                  texture.data);
            // }

//            // select one texture for each triangle
//            std::vector<int> tex4tri;
//            for (list<dlovi::Matrix>::const_iterator it = GetTris().begin(); it != GetTris().end(); it++) {
//                int tex_ind_curr = -1;
//
//                for (int i = 0; i < numKFs; i++) {
//                    dlovi::Matrix point0 = GetPoints()[(*it)(0)];
//                    dlovi::Matrix point1 = GetPoints()[(*it)(1)];
//                    dlovi::Matrix point2 = GetPoints()[(*it)(2)];
//
//                    TextureFrame tex = imAndTexFrame[i].second;
//                    vector<float> uv0 = tex.GetTexCoordinate(point0(0), point0(1), point0(2), imSize);
//                    vector<float> uv1 = tex.GetTexCoordinate(point1(0), point1(1), point1(2), imSize);
//                    vector<float> uv2 = tex.GetTexCoordinate(point2(0), point2(1), point2(2), imSize);
//
//                    if (uv0.size() == 2 && uv1.size() == 2 && uv2.size() == 2) {
//                        tex_ind_curr = i;
//                        break;
//                    }
//                }
//
//                if (tex_ind_curr < 0)
//                    tex_ind_curr = 0;
//
//                tex4tri.push_back(tex_ind_curr);
//            }


            glBegin(GL_TRIANGLES);
            glColor3f(1.0, 1.0, 1.0);

            int index = 0;
            for (list<dlovi::Matrix>::const_iterator it = GetTris().begin(); it != GetTris().end(); it++, index++) {

                dlovi::Matrix point0 = GetPoints()[(*it)(0)];
                dlovi::Matrix point1 = GetPoints()[(*it)(1)];
                dlovi::Matrix point2 = GetPoints()[(*it)(2)];
                // glVertex3d(point0(0), point0(1), point0(2));
                // glVertex3d(point1(0), point1(1), point1(2));
                // glVertex3d(point2(0), point2(1), point2(2));
                for (int i = 0; i < numKFs; i++) {
                    vector<float> uv0 = vpKFtex[i]->GetTexCoordinate(point0(0), point0(1), point0(2), imSize);
                    vector<float> uv1 = vpKFtex[i]->GetTexCoordinate(point1(0), point1(1), point1(2), imSize);
                    vector<float> uv2 = vpKFtex[i]->GetTexCoordinate(point2(0), point2(1), point2(2), imSize);

                    if (uv0.size() == 2 && uv1.size() == 2 && uv2.size() == 2) {

                        glTexCoord2f(uv0[0], (uv0[1]+i)/numKFs);
                        glVertex3d(point0(0), point0(1), point0(2));

                        glTexCoord2f(uv1[0], (uv1[1]+i)/numKFs);
                        glVertex3d(point1(0), point1(1), point1(2));

                        glTexCoord2f(uv2[0], (uv2[1]+i)/numKFs);
                        glVertex3d(point2(0), point2(1), point2(2));

                        break;

                    }
                }

            }
            glEnd();

            glDisable(GL_TEXTURE_2D);

        }

        for (auto it = vpKFtex.begin(); it != vpKFtex.end(); it++)
            (*it)->SetEraseDrawer(); //release the keyframes

    }

    void ModelDrawer::DrawModelPoints()
    {
        UpdateModel();

        glPointSize(3);
        glBegin(GL_POINTS);
        glColor3f(0.5, 0.5, 0.5);
        //cout<<"model point size: "<<GetPoints().size()<<endl;
        for (size_t i = 0; i < GetPoints().size(); i++) {
            glVertex3d(GetPoints()[i](0), GetPoints()[i](1), GetPoints()[i](2));
        }
        glEnd();
    }

    void ModelDrawer::SetTarget(pangolin::OpenGlMatrix &Twc, Eigen::Vector3d t)
    {
      Eigen::Vector3d diff = t - target;
      if (diff.squaredNorm() > 0.0) {
          target = t;
          //Eigen::Matrix<double,4,4> m = Twc;
          //target_w = Eigen::Transform<double,3,Eigen::Affine>(m) * t;
          target_w = t;
          //search all map points, find nearest target_w, only in camera's x, y plane.
          double minDist = 99999;
          int nearestIndex = -1;
          for (size_t i = 0; i < GetPoints().size(); i++) {
            double distance = sqrt((GetPoints()[i](0)-target_w[0])*(GetPoints()[i](0)-target_w[0])+
            (GetPoints()[i](1)-target_w[1])*(GetPoints()[i](1)-target_w[1]) +
            (GetPoints()[i](2)-target_w[2])*(GetPoints()[i](2)-target_w[2]));

            if(distance<minDist)
            {
              nearestIndex = i;
              minDist = distance;
            }
          }
          target_w[0] = GetPoints()[nearestIndex](0);
          target_w[1] = GetPoints()[nearestIndex](1);
          target_w[2] = GetPoints()[nearestIndex](2);
          // const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
          // if(vpMPs.empty())
          //     return;
          //
          // for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
          // {
          //     // if(vpMPs[i]->isBad())
          //     //     continue;
          //     cv::Mat pos = vpMPs[i]->GetWorldPos();
          //     double distance = sqrt((pos.at<double>(0)-target_w[0])*(pos.at<double>(0)-target_w[0])+
          //     (pos.at<double>(1)-target_w[1])*(pos.at<double>(1)-target_w[1])+
          //     (pos.at<double>(2)-target_w[2])*(pos.at<double>(2)-target_w[2]));
          //     if(distance<minDist)
          //     {
          //       nearestIndex = i;
          //       minDist = distance;
          //     }
          // }
          // target_w[0] = (vpMPs[nearestIndex]->GetWorldPos()).at<double>(0);
          // target_w[1] = (vpMPs[nearestIndex]->GetWorldPos()).at<double>(1);
          // target_w[2] = (vpMPs[nearestIndex]->GetWorldPos()).at<double>(2);
      }
      //cout<<"selected target target_w: "<<target_w<<endl;
    }

    cv::Mat ModelDrawer::GetTarget()
    {
        cv::Mat T = (cv::Mat_<float>(3,1) << target_w[0], target[1], target[2]);
        return T;
    }

    string ModelDrawer::GetTargetStr()
    {
      std::stringstream ss;
      ss<<target_w[0]<<","<<target_w[1]<<","<<target_w[2];
      return ss.str();
    }

    void ModelDrawer::DrawTarget()
    {
        glPointSize(10);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(target_w[0], target_w[1], target_w[2]);
        glEnd();
    }

    void ModelDrawer::DrawTarget(pangolin::OpenGlMatrix &Twc, Eigen::Vector3d t)
    {
        SetTarget(Twc, t);
        DrawTarget();
    }

    void ModelDrawer::DrawTriangles(pangolin::OpenGlMatrix &Twc)
    {
        UpdateModel();

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        GLfloat light_position[] = { 0.0, 0.0, 1.0, 0.0 };
        glLightfv(GL_LIGHT0, GL_POSITION, light_position);

        glPopMatrix();

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);

        glShadeModel(GL_FLAT);

        GLfloat material_diffuse[] = {0.2, 0.5, 0.8, 1};
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_diffuse);

        glBegin(GL_TRIANGLES);
        glColor3f(1.0,1.0,1.0);

        for (list<dlovi::Matrix>::const_iterator it = GetTris().begin(); it != GetTris().end(); it++) {

            dlovi::Matrix point0 = GetPoints()[(*it)(0)];
            dlovi::Matrix point1 = GetPoints()[(*it)(1)];
            dlovi::Matrix point2 = GetPoints()[(*it)(2)];

            dlovi::Matrix edge10 = point1 - point0;
            dlovi::Matrix edge20 = point2 - point0;

            dlovi::Matrix normal = edge20.cross(edge10);
            normal = normal / normal.norm();

            glNormal3d(normal(0), normal(1), normal(2));

            glVertex3d(point0(0), point0(1), point0(2));
            glVertex3d(point1(0), point1(1), point1(2));
            glVertex3d(point2(0), point2(1), point2(2));

        }
        glEnd();

        glDisable(GL_LIGHTING);

    }

    void ModelDrawer::DrawFrame(bool bRGB)
    {
        // select the last frame
        int numKFs = 1;

        vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

        std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        vector<KeyFrame*> vpKFtex;

        for(auto it = vpKFs.rbegin(); it != vpKFs.rend(); ++it) {
            if ((int)vpKFtex.size() >= numKFs)
                break;
            KeyFrame *kf = *it;
            kf->SetNotEraseDrawer();
            if (kf->isBad()) {
                kf->SetEraseDrawer();
                continue;
            }
            vpKFtex.push_back(kf);
        }


        if ((int)vpKFtex.size() >= numKFs) {
            glColor3f(1.0,1.0,1.0);

            if (vpKFtex[0]->mImage.empty()){
                std::cerr << "ERROR: empty frame image" << endl;
                return;
            }
            cv::Size imSize = vpKFtex[0]->mImage.size();

            if(bRGB) {
                pangolin::GlTexture imageTexture(imSize.width, imSize.height, GL_RGB, false, 0, GL_BGR,
                                                 GL_UNSIGNED_BYTE);
                imageTexture.Upload(vpKFtex[0]->mImage.data, GL_BGR, GL_UNSIGNED_BYTE);
                imageTexture.RenderToViewportFlipY();
            } else {
                pangolin::GlTexture imageTexture(imSize.width, imSize.height, GL_RGB, false, 0, GL_RGB,
                                                 GL_UNSIGNED_BYTE);
                imageTexture.Upload(vpKFtex[0]->mImage.data, GL_RGB, GL_UNSIGNED_BYTE);
                imageTexture.RenderToViewportFlipY();
            }

        }
    }

    void ModelDrawer::UpdateModel()
    {
        if(mbModelUpdateRequested && ! mbModelUpdateDone)
            return;

        if(mbModelUpdateRequested && mbModelUpdateDone){
            mModel = mUpdatedModel;
            mbModelUpdateRequested = false;
            return;
        }

        mbModelUpdateDone = false;
        mbModelUpdateRequested = true; // implicitly signals SurfaceInferer thread which is polling
    }

    void ModelDrawer::SetUpdatedModel(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris)
    {
        mUpdatedModel.first = modelPoints;
        mUpdatedModel.second = modelTris;
        mbModelUpdateRequested = true;
    }

    vector<dlovi::Matrix> & ModelDrawer::GetPoints()
    {
        return mModel.first;
    }

    list<dlovi::Matrix> & ModelDrawer::GetTris()
    {
        return mModel.second;
    }

    void ModelDrawer::MarkUpdateDone()
    {
        mbModelUpdateDone = true;
    }

    bool ModelDrawer::UpdateRequested()
    {
        return mbModelUpdateRequested;
    }

    bool ModelDrawer::UpdateDone()
    {
        return mbModelUpdateDone;
    }

    void ModelDrawer::SetModeler(Modeler* pModeler)
    {
        mpModeler = pModeler;
    }

} //namespace ORB_SLAM
