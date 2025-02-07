//
// Created by shida on 06/12/16.
//

#ifndef __MODELDRAWER_H
#define __MODELDRAWER_H

#include<pangolin/pangolin.h>

#include<mutex>

#include <map>
#include <list>
#include <deque>
#include <vector>
#include "Modeler/Matrix.h"
#include "Modeler/Modeler.h"
#include "Map.h"

namespace ORB_SLAM2
{

    class KeyFrame;
    class Modeler;

    class ModelDrawer
    {
    public:
        ModelDrawer(Map* pMap);

        void DrawModel(bool bRGB);
        void DrawModelPoints();
        void DrawTriangles(pangolin::OpenGlMatrix &Twc);
        void DrawFrame(bool bRGB);

        void SetTarget(pangolin::OpenGlMatrix &Twc, Eigen::Vector3d t);
        cv::Mat GetTarget();
        string GetTargetStr();
        void DrawTarget(pangolin::OpenGlMatrix &Twc, Eigen::Vector3d t);
        void DrawTarget();

        void UpdateModel();
        void SetUpdatedModel(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris);

        void MarkUpdateDone();
        bool UpdateRequested();
        bool UpdateDone();

        vector<dlovi::Matrix> & GetPoints();
        list<dlovi::Matrix> & GetTris();

        void SetModeler(Modeler* pModeler);

    private:

        Map* mpMap;

        Modeler* mpModeler;

        bool mbModelUpdateRequested;
        bool mbModelUpdateDone;

        std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix>> mModel;
        std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix>> mUpdatedModel;

        Eigen::Vector3d target;
        Eigen::Vector3d target_w;
    };

} //namespace ORB_SLAM

#endif //__MODELDRAWER_H
