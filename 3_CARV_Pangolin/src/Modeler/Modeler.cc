//
// Created by shida on 05/12/16.
//

#include "Modeler/Modeler.h"

#include <ctime>

#include <chrono>


namespace ORB_SLAM2 {

    Modeler::Modeler(ModelDrawer* pModelDrawer):
            mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpModelDrawer(pModelDrawer),
            mnLastNumLines(2), mbFirstKeyFrame(true)
    {
        mAlgInterface.setAlgorithmRef(&mObjAlgorithm);
        mAlgInterface.setTranscriptRef(mTranscriptInterface.getTranscriptToProcessRef());
        mAlgInterface.rewind();
        commandLineEnterBlock = false;
    }

    void Modeler::SetTracker(Tracking *pTracker)
    {
        mpTracker=pTracker;
    }

    void Modeler::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper=pLocalMapper;
    }

    void Modeler::SetLoopCloser(LoopClosing* pLoopCloser)
    {
        mpLoopCloser = pLoopCloser;
    }

    void Modeler::Run()
    {
        mbFinished =false;

        while(1) {

            if (CheckNewTranscriptEntry()) {

                // RunRemainder();
                //
                // UpdateModelDrawer();
            }

            ResetIfRequested();

            if(CheckFinish())
                break;

            usleep(1000);
        }

//        //CARV
//        {
//            unique_lock<mutex> lock(mMutexTranscript);
//            std::cout << std::endl << "Saving transcript to file ..." << std::endl;
//            mTranscriptInterface.writeToFile("sfmtranscript_orbslam.txt");
//            std::cout << std::endl << "transcript saved!" << std::endl;
//        }
//
        SetFinish();

    }

    void Modeler::writeToFile(const std::string & strFileName) const{
            mTranscriptInterface.writeToFile(strFileName);
    }

    void Modeler::addCommandLine(const std::string & line)
    {
        unique_lock<mutex> lock(mMutexTranscript);
        mAlgInterface.m_pTranscript->addLine(line);
        if(line.find("{") != std::string::npos)
          commandLineEnterBlock = true;
        else if(line.find("}") != std::string::npos)
          commandLineEnterBlock = false;

        if(!commandLineEnterBlock)
        {
          mAlgInterface.step(true);
          //cout<<count<<"  step done"<<endl;
          std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > objModel = mAlgInterface.getCurrentModel();
          cout<<" objModel first count "<<objModel.first.size()<<"  objModel second count "<<objModel.second.size()<<endl;

          mpModelDrawer->SetUpdatedModel(objModel.first, objModel.second);
          mpModelDrawer->MarkUpdateDone();
          mpModelDrawer->UpdateModel();
          cout<<" getpoints num: "<<mpModelDrawer->GetPoints().size()<<endl;
          cout<<"  update drawer done"<<endl;
          //getchar();
        }
    }

    void Modeler::addCommandBlock(const std::string & block)
    {
        unique_lock<mutex> lock(mMutexTranscript);
        if(block.find("@") != std::string::npos)
        {
            std::vector<std::string> arrStr = split(block, "@");
            for(int i=0;i<arrStr.size();i++)
              mAlgInterface.m_pTranscript->addLine(arrStr[i]);
        }
        else
          mAlgInterface.m_pTranscript->addLine(block);

        mAlgInterface.step(true);
        //cout<<count<<"  step done"<<endl;
        std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > objModel = mAlgInterface.getCurrentModel();
        cout<<" objModel first count "<<objModel.first.size()<<"  objModel second count "<<objModel.second.size()<<endl;

        mpModelDrawer->SetUpdatedModel(objModel.first, objModel.second);
        mpModelDrawer->MarkUpdateDone();
        mpModelDrawer->UpdateModel();
        cout<<" getpoints num: "<<mpModelDrawer->GetPoints().size()<<endl;
        cout<<"  update drawer done"<<endl;
        //getchar();
    }


    void Modeler::loadModelFromFile(const std::string & strFileName){
      std::string strTmp;
      std::ifstream fileIn(strFileName.c_str(), std::ios::in);
      cout<<"File Loaded"<<endl;
      std::stringstream ssTmp;
      bool enterBlock = false;
      int count = 0;
      while(std::getline(fileIn, strTmp))
      {
        cout<<"count "<<count<<endl;
        addCommandLine(strTmp);
        count ++;
        // unique_lock<mutex> lock(mMutexTranscript);
        //   mAlgInterface.m_pTranscript->addLine(strTmp);
        //   cout<<"count "<<count<<" Add line done"<<endl;
        //   count++;
        //   // //load until the next }
        //   // ssTmp <<strTmp;
        //   // //mTranscriptInterface.m_SFMTranscript.addLine(strTmp);
        //   //
        //   if(strTmp.find("{") != std::string::npos)
        //     enterBlock = true;
        //   else if(strTmp.find("}") != std::string::npos)
        //     enterBlock = false;
        //
        //   if(enterBlock)
        //   {
        //     //ssTmp<<"@";
        //     continue;
        //   }
        //   else
        //   {
        //     cout<<count<<"  step start"<<endl;
        //     mAlgInterface.step(true);
        //     cout<<count<<"  step done"<<endl;
        //     std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > objModel = mAlgInterface.getCurrentModel();
        //     cout<<count<<" objModel first count "<<objModel.first.size()<<"  objModel second count "<<objModel.second.size()<<endl;
        //
        //     mpModelDrawer->SetUpdatedModel(objModel.first, objModel.second);
        //     mpModelDrawer->MarkUpdateDone();
        //     mpModelDrawer->UpdateModel();
        //     cout<<count<<" getpoints num: "<<mpModelDrawer->GetPoints().size()<<endl;
        //     cout<<count<<"  update drawer done"<<endl;
        //     //getchar();
        //   }
        //   //
        //   // //cout<<"single command: "<<strTmp<<endl;
        //   // //if(strTmp.find("}") != std::string::npos)
        //   // //mAlgInterface.step();
        //   // //UpdateModelDrawer();
        //   // parseSingleCommandStr(ssTmp.str());
        //   // ssTmp.str("");
      }

      fileIn.close();
      //mAlgInterface.step();
    }

    void Modeler::parseSingleCommandStr(const std::string & line){
      cout<<"single command: "<<line<<endl;
      (mAlgInterface.m_pTranscript)->stepTranscriptCommandStr(line);
      cout<<"mark 2"<<endl;
      //mAlgInterface.step();
    }

    void Modeler::UpdateModelDrawer() {
        if(mpModelDrawer->UpdateRequested() && ! mpModelDrawer->UpdateDone()) {
            std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > objModel = mAlgInterface.getCurrentModel();
            mpModelDrawer->SetUpdatedModel(objModel.first, objModel.second);
            mpModelDrawer->MarkUpdateDone();
        }
    }

    bool Modeler::CheckNewTranscriptEntry()
    {
        unique_lock<mutex> lock(mMutexTranscript);
        int numLines = mTranscriptInterface.getTranscriptRef()->numLines();
        if (numLines > mnLastNumLines) {
            mnLastNumLines = numLines;
            mTranscriptInterface.UpdateTranscriptToProcess();
            return true;
        } else {
            return false;
        }
    }

    void Modeler::RunRemainder()
    {
        mAlgInterface.runRemainder();
    }

    void Modeler::AddKeyFrameEntry(KeyFrame* pKF){
        if(pKF->isBad())
            return;

        // Avoid that a keyframe can be erased while it is being process by this thread
        pKF->SetNotErase();

        if (mbFirstKeyFrame) {
            unique_lock<mutex> lock(mMutexTranscript);
            mTranscriptInterface.addFirstKeyFrameInsertionEntry(pKF);
            mbFirstKeyFrame = false;
        } else {
            unique_lock<mutex> lock(mMutexTranscript);
            mTranscriptInterface.addKeyFrameInsertionEntry(pKF);
        }

        pKF->SetErase();
    }

    void Modeler::AddDeletePointEntry(MapPoint* pMP){
        unique_lock<mutex> lock(mMutexTranscript);
        mTranscriptInterface.addPointDeletionEntry(pMP);
    }

    void Modeler::AddDeleteObservationEntry(KeyFrame *pKF, MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexTranscript);
        mTranscriptInterface.addVisibilityRayDeletionEntry(pKF, pMP);
    }

    void Modeler::AddAdjustmentEntry(std::set<KeyFrame*> & sAdjustSet, std::set<MapPoint*> & sMapPoints){
        unique_lock<mutex> lock(mMutexTranscript);
        mTranscriptInterface.addBundleAdjustmentEntry(sAdjustSet, sMapPoints);
    }


    void Modeler::RequestReset()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetRequested = true;
        }

        while(1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if(!mbResetRequested)
                    break;
            }
            usleep(100);
        }
    }

    void Modeler::ResetIfRequested()
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            {
                unique_lock<mutex> lock2(mMutexTranscript);
//                mTranscriptInterface.writeToFile("sfmtranscript_orbslam.txt");
                mTranscriptInterface.addResetEntry();
                //TODO: fix crash when initialize again after reset
//            RunRemainder();
//            mAlgInterface.rewind();
            }

            mbFirstKeyFrame = true;

            mbResetRequested=false;
        }

    }

    void Modeler::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Modeler::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Modeler::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Modeler::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

}
