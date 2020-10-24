//
// Created by fishmarch on 20-6-22.
//

#include "MapPlane.h"


#include <mutex>
#include <time.h>

namespace ORB_SLAM2 {
    long unsigned int MapPlane::nLastId = 0;
    int MapPlane::mnObserveTh = 30;
    mutex MapPlane::mGlobalMutex;

    MapPlane::MapPlane(const cv::Mat &Pos, ORB_SLAM2::KeyFrame *pRefKF, Map *pMap) :
            mnBALocalForKF(0), mpMap(pMap), mpRefKF(pRefKF),mbBad(false) {
        Pos.copyTo(mWorldPos);
        mnId = nLastId++;
        if(mnId==1) {
            srand(time(0));
            mnObserveTh = Config::Get<int>("MapPlane.ObserveTimes");
        }
        mbBad = true;
        mbBadPre = true;
        bool bsetting = true;
        while(bsetting){
            mRed = rand() % 255;
            mBlue = rand() % 255;
            mGreen = rand() % 255;
            if(mRed*0.299 + mGreen*0.578 + mBlue*0.114 >= 192){
                bsetting = false;
            }
        }

    }

    void MapPlane::AddObservation(ORB_SLAM2::KeyFrame *pKF, int idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return;
        mObservations[pKF] = idx;
        if(mObservations.size() > 3)
            if(!mbBadPre)
                mbBad = false;
    }

    void MapPlane::AddEndPoints(pair<cv::Mat,cv::Mat> pts){
        unique_lock<mutex> lock(mMutexFeatures);
        mvEndPoints.push_back(pts);
    }

    cv::Mat MapPlane::GetWorldPos(){
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos.clone();
    }

    std::vector<pair<cv::Mat,cv::Mat>> MapPlane::GetAllEndPoints(){
        unique_lock<mutex> lock(mMutexFeatures);
        return mvEndPoints;
    }

    map<ORB_SLAM2::KeyFrame*, int> MapPlane::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    void MapPlane::EraseObservation(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF)){
            mObservations.erase(pKF);
        }
        if(mObservations.size() < 2)
            mbBad = true;
    }

    void MapPlane::AddFrameObservation(Frame& pF) {
        unique_lock<mutex> lock(mMutexFeatures);
        mObserveFrames.insert(pF.mnId);
        if(mObserveFrames.size() > mnObserveTh) {
            mbBadPre = false;
        }
    }

    int MapPlane::GetIndexInKeyFrame(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    void MapPlane::SetWorldPos(const cv::Mat &Pos)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        Pos.copyTo(mWorldPos);
    }

    cv::Mat MapPlane::GetPoseInFrame(ORB_SLAM2::Frame &pF) {
        unique_lock<mutex> lock(mMutexPos);
        cv::Mat temp;
        cv::transpose(pF.mTwc, temp);
        return temp*mWorldPos;
    }
    cv::Mat MapPlane::GetPoseInFrame(ORB_SLAM2::Frame *pF) {
        unique_lock<mutex> lock(mMutexPos);
        cv::Mat temp;
        cv::transpose(pF->mTwc, temp);
        return temp*mWorldPos;
    }
}