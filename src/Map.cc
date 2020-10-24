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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
    mfDisTh = Config::Get<float>("Match.DistanceTh");
    mfAngleTh = Config::Get<float>("Match.AngleTh");
    mfAngleTh = cos(mfAngleTh/180*M_PI);
    mfDisTh = mfDisTh/100;
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

    void Map::AddMapPlane(MapPlane *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mvpMapPlanes.push_back(pMP);
    }


void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

    vector<MapPlane*> Map::GetAllMapPlanes() {
        unique_lock<mutex> lock(mMutexMap);
        return mvpMapPlanes;
    }

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}
    void Map::MapPlaneMatching(ORB_SLAM2::Frame &pF) {
        unique_lock<mutex> lock(mMutexMap);
        if(mvpMapPlanes.empty())
            return;
        pF.mbNewPlane = false;
        pF.mbKeyFramePlane = false;
        for(int i=0; i<pF.mvPlanes.size(); i++){
            cv::Mat pM = pF.ComputeCameraPlane(i);
            pF.mvpMapPlanes[i] = static_cast<MapPlane*>(nullptr);
            int nlinei = pF.mvPlaneLineNo[i].first;
            int nlinej = pF.mvPlaneLineNo[i].second;
            cv::Point3d p3Dis = pF.mvLineEndPoints[nlinei].first;
            cv::Point3d p3Die = pF.mvLineEndPoints[nlinei].second;
            cv::Point3d p3Djs = pF.mvLineEndPoints[nlinej].first;
            cv::Point3d p3Dje = pF.mvLineEndPoints[nlinej].second;
            float ldTh = mfDisTh;
            float lATh = mfAngleTh;
            int nsearchNum = 100;
            int nMaxPlaneMatchTimes = -1;
            for(auto sit=mvpMapPlanes.end()-1, send=mvpMapPlanes.begin(); sit!=send && nsearchNum > 0; sit--){
                nsearchNum--;
                cv::Mat pW = (*sit)->GetPoseInFrame(pF); // To camera coordinates
                if(pW.at<float>(3, 0) < 0){
                    pW = -pW;
                }
                float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                              pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                              pM.at<float>(2, 0) * pW.at<float>(2, 0);
                if ((angle > lATh)){
                    float dd = pM.at<float>(3, 0) - pW.at<float>(3, 0);
                    float disis = (pW.at<float>(0, 0) * p3Dis.x + pW.at<float>(1, 0) * p3Dis.y + pW.at<float>(2, 0) * p3Dis.z + pW.at<float>(3, 0));
                    float disie = (pW.at<float>(0, 0) * p3Die.x + pW.at<float>(1, 0) * p3Die.y + pW.at<float>(2, 0) * p3Die.z + pW.at<float>(3, 0));
                    float disjs = (pW.at<float>(0, 0) * p3Djs.x + pW.at<float>(1, 0) * p3Djs.y + pW.at<float>(2, 0) * p3Djs.z + pW.at<float>(3, 0));
                    float disje = (pW.at<float>(0, 0) * p3Dje.x + pW.at<float>(1, 0) * p3Dje.y + pW.at<float>(2, 0) * p3Dje.z + pW.at<float>(3, 0));
                    dd = (disis + disie + disjs + disje)/4;
                    if(dd < ldTh && dd > -ldTh){
                        pF.mvpMapPlanes[i] = (*sit);
                        ldTh = abs(dd);
                        lATh = angle;
                    }
                }
            }
        }
        for(auto p : pF.mvpMapPlanes){
            if(p)
                p->AddFrameObservation(pF);
        }
    }
} //namespace ORB_SLAM


