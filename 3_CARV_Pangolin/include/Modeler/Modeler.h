
#ifndef __MODELER_H
#define __MODELER_H

#include <mutex>

#include "Modeler/SFMTranscriptInterface_ORBSLAM.h"
#include "Modeler/SFMTranscriptInterface_Delaunay.h"
#include "Modeler/ModelDrawer.h"

namespace ORB_SLAM2 {

    class Tracking;
    class LoopClosing;
    class LocalMapping;
    class KeyFrame;
    class Frame;
    class ModelDrawer;

    class Modeler {
    public:
        Modeler(ModelDrawer* pModelDrawer);

        std::vector<std::string> split(const std::string & strOriginal, const std::string & strDelimiter){
          vector<string> retVal;
          {
            string::size_type nOriginalLength = strOriginal.length();
            string::size_type nDelimiterLength = strDelimiter.length();
            string::size_type nFirst = 0;
            string::size_type nSecond;

            while(nFirst < nOriginalLength){
              nSecond = strOriginal.find(strDelimiter, nFirst);
              if(nSecond == string::npos){
                retVal.push_back(strOriginal.substr(nFirst));
                return retVal;
              }
              retVal.push_back(strOriginal.substr(nFirst, nSecond - nFirst));
              nFirst = nSecond + nDelimiterLength;
            }

            return retVal;
          }

          return retVal;
        };

        void SetLoopCloser(LoopClosing* pLoopCloser);

        void SetLocalMapper(LocalMapping* pLocalMapper);

        void SetTracker(Tracking* pTracker);

        // Main function
        void Run();

        void UpdateModelDrawer();
        bool CheckNewTranscriptEntry();
        void RunRemainder();

        void AddKeyFrameEntry(KeyFrame* pKF);
        void AddDeletePointEntry(MapPoint* pMP);
        void AddDeleteObservationEntry(KeyFrame* pKF, MapPoint* pMP);
        void AddAdjustmentEntry(std::set<KeyFrame*> & sAdjustSet, std::set<MapPoint*> & sMapPoints);

        // Thread Synch
        void RequestReset();
        void RequestFinish();
        bool isFinished();
        void parseSingleCommandStr(const std::string & line);
        void writeToFile(const std::string & strFileName) const;
        void loadModelFromFile(const std::string & strFileName);

        void addCommandLine(const std::string & line);
        void addCommandBlock(const std::string & block);
    public:
        bool commandLineEnterBlock;
        void ResetIfRequested();
        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        Tracking* mpTracker;
        LocalMapping* mpLocalMapper;
        LoopClosing* mpLoopCloser;

        ModelDrawer* mpModelDrawer;

        // This avoid that two transcript entries are created simultaneously in separate threads
        std::mutex mMutexTranscript;

        //number of lines in transcript last time checked
        int mnLastNumLines;

        //CARV interface
        SFMTranscriptInterface_ORBSLAM mTranscriptInterface; // An interface to a transcript / log of the map's work.
        //CARV runner instance
        dlovi::FreespaceDelaunayAlgorithm mObjAlgorithm;
        SFMTranscriptInterface_Delaunay mAlgInterface; // An encapsulation of the interface between the transcript and the surface inferring algorithm.
        bool mbFirstKeyFrame;

    };
}


#endif //__MODELVIEWER_H
