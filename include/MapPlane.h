//
// Created by fishmarch on 20-6-22.
//

#ifndef ORB_SLAM2_MAPPLANE_H
#define ORB_SLAM2_MAPPLANE_H
#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include "Converter.h"
#include <opencv2/core/core.hpp>
#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>


namespace ORB_SLAM2 {
    class KeyFrame;
    class Frame;
    class Map;
    class MapPlane {
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud <PointT> PointCloud;
    public:
        MapPlane(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);

        void SetWorldPos(const cv::Mat &Pos);
        cv::Mat GetWorldPos();

        void AddObservation(KeyFrame* pKF, int idx);
        void EraseObservation(KeyFrame* pKF);

        map<KeyFrame*, int> GetObservations();

        int GetIndexInKeyFrame(KeyFrame *pKF);

        void AddEndPoints(pair<cv::Mat,cv::Mat> pts);
        std::vector<pair<cv::Mat,cv::Mat>> GetAllEndPoints();

        void AddFrameObservation(Frame& pF);

        cv::Mat GetPoseInFrame(Frame& pF);
        cv::Mat GetPoseInFrame(Frame* pF);

    public:
        long unsigned int mnId; ///< Global ID for MapPlane;
        static long unsigned int nLastId;
        static std::mutex mGlobalMutex;
        long unsigned int mnBALocalForKF; //used in local BA

        //used for visualization
        int mRed;
        int mGreen;
        int mBlue;

        bool mbBad;
        bool mbBadPre;
        std::set<int> mObserveFrames;

    protected:
        cv::Mat mWorldPos; ///< Position in absolute coordinates
        std::map<KeyFrame*, int> mObservations;
        std::mutex mMutexPos;
        std::mutex mMutexFeatures;

        Map* mpMap;
        // Bad flag (we do not currently erase MapPoint from memory)

        MapPlane* mpReplaced;

        // Reference KeyFrame
        KeyFrame* mpRefKF;

        std::vector<pair<cv::Mat,cv::Mat>> mvEndPoints;

        static int mnObserveTh;
    };
}

#endif //ORB_SLAM2_MAPPLANE_H
