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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
int Frame::mnLineMinLength = 0;
int Frame::mnLineMaxNumber = 50;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;
float Frame::mfParallelAngle, Frame::mfSamePlaneDis;
float Frame::mfDisTh, Frame::mfAngleTh;
bool Frame::mbUseParallelLines;
Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
     mvKeys_Line(frame.mvKeys_Line), mvKeysRight_Line(frame.mvKeysRight_Line),mvLine3DDirections(frame.mvLine3DDirections),
     mvLineEndPoints(frame.mvLineEndPoints), mvPlanes(frame.mvPlanes), mvPlaneLineNo(frame.mvPlaneLineNo),mbKeyFramePlane(frame.mbKeyFramePlane),
     mbNewPlane(frame.mbNewPlane)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    thread threadLeft_Line(&Frame::ExtractLine,this,0,imLeft);
    thread threadRight_Line(&Frame::ExtractLine,this,1,imRight);
    threadLeft.join();
    threadRight.join();
    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoMatches();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;

        mnLineMinLength = Config::Get<int>("Line.MinSize");
        mnLineMaxNumber = Config::Get<int>("Line.MaxNumber");
        mfParallelAngle = Config::Get<float>("Line.ParallelAngle");
        mfParallelAngle = cos(mfParallelAngle/180*M_PI);
        mfSamePlaneDis = Config::Get<float>("Line.SamePlaneDis");
        mfSamePlaneDis = mfSamePlaneDis/100;
        mbUseParallelLines = (bool)Config::Get<int>("Line.UseParallelLines");
        mfDisTh = Config::Get<float>("Match.DistanceTh");
        mfAngleTh = Config::Get<float>("Match.AngleTh");
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();

    inv_width  = FRAME_GRID_COLS / static_cast<double>(imLeft.cols);
    inv_height = FRAME_GRID_ROWS / static_cast<double>(imRight.rows);

    threadLeft_Line.join();
    threadRight_Line.join();

    ComputeStereoMatches_Lines();
    ComputePlansFromLines();
    mvpMapPlanes = vector<MapPlane*>(mvPlanes.size(),static_cast<MapPlane*>(NULL));
    mvbPlaneOutlier = vector<bool>(mvPlanes.size(), false);
    mbNewPlane = false;
    mbKeyFramePlane = false;
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tt= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    ORB_SLAM2::Timer::SetTPlane(tt);

}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}
    struct sort_lines_by_response {
        inline bool operator()(const cv::line_descriptor::KeyLine &a, const cv::line_descriptor::KeyLine &b) {
            return (a.response > b.response);
        }
    };

    void Frame::ExtractLine(int flag, const cv::Mat &im)
    {
        cv::Ptr<cv::line_descriptor::BinaryDescriptor> lbd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
        cv::Ptr<cv::line_descriptor::LSDDetectorC> lsd = cv::line_descriptor::LSDDetectorC::createLSDDetectorC();
        int lsdNFeatures = mnLineMaxNumber;
        cv::line_descriptor::LSDDetectorC::LSDOptions opts;
        opts.refine       = 0;
        opts.scale        = 1.2;
        opts.sigma_scale  = 0.6;
        opts.quant        = 2.0;
        opts.ang_th       = 22.5;
        opts.log_eps      = 1.0;
        opts.density_th   = 0.6;
        opts.n_bins       = 1024;
        opts.min_length   = mnLineMinLength;
        if(flag==0) {
            lsd->detect(im, mvKeys_Line, 1.2, 1, opts);
            if(mvKeys_Line.size()>lsdNFeatures)
            {
                sort(mvKeys_Line.begin(), mvKeys_Line.end(), sort_lines_by_response());
                mvKeys_Line.resize(lsdNFeatures);
                for( int i=0; i<lsdNFeatures; i++)
                    mvKeys_Line[i].class_id = i;
            }
            lbd->compute(im, mvKeys_Line, mDescriptors_Line);
        }else{
            lsd->detect(im, mvKeysRight_Line, 1.2, 1, opts);
            if(mvKeysRight_Line.size()>lsdNFeatures)
            {
                sort(mvKeysRight_Line.begin(), mvKeysRight_Line.end(), sort_lines_by_response());
                mvKeysRight_Line.resize(lsdNFeatures);
                for( int i=0; i<lsdNFeatures; i++)
                    mvKeysRight_Line[i].class_id = i;
            }
            lbd->compute(im, mvKeysRight_Line, mDescriptorsRight_Line);
        }
    }

    void Frame::FastExtractLine(int flag, const cv::Mat &im)
    {

        int    length_threshold    = mnLineMinLength;
        float  distance_threshold  = 1.41421356f;
        double canny_th1           = 50.0;
        double canny_th2           = 50.0;
        int    canny_aperture_size = 3;
        bool   do_merge            = true;
        cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(
                length_threshold,
                distance_threshold,
                canny_th1,
                canny_th2,
                canny_aperture_size,
                do_merge);

        if(flag==0) {
            fld->detect(im,mvLinesLeft);
        }else {
            fld->detect(im,mvLinesRight);
        }
    }

    void Frame::ComputeStereoMatches_Lines()
    {
        mvDisparity_l.clear();
        mvDisparity_l.resize(mvKeys_Line.size(),pair<float,float>(-1,-1));
        mvLineEndPoints.clear();
        mvLineEndPoints.resize(mvKeys_Line.size(),pair<cv::Point3d,cv::Point3d>((0,0,0),(0,0,0)));
        mvLine3DDirections.clear();
        mvLine3DDirections.resize(mvKeys_Line.size(), cv::Point3d(0,0,0));


        // Line segments stereo matching
        // --------------------------------------------------------------------------------------------------------------------
        if (mvKeys_Line.empty() || mvKeysRight_Line.empty())
            return;

        std::vector<line_2d> coords; // int
        coords.reserve(mvKeys_Line.size()); // coordinate in grid
        for (const auto &kl : mvKeys_Line)
            coords.push_back(std::make_pair(std::make_pair(kl.startPointX * inv_width, kl.startPointY * inv_height),
                                            std::make_pair(kl.endPointX * inv_width, kl.endPointY * inv_height)));

        //Fill in grid & directions
        list<pair<int, int>> line_coords;
        GridStructure grid(FRAME_GRID_ROWS, FRAME_GRID_COLS);
        std::vector<std::pair<double, double>> directions(mvKeysRight_Line.size());
        for(unsigned int idx = 0; idx < mvKeysRight_Line.size(); ++idx) {
            const auto &kl = mvKeysRight_Line[idx];
            std::pair<double, double> &v = directions[idx];
            v = std::make_pair((kl.endPointX - kl.startPointX) * inv_width, (kl.endPointY - kl.startPointY) * inv_height);
            normalize(v);
            //Get all points on line in grid coordinate
            getLineCoords(kl.startPointX * inv_width, kl.startPointY * inv_height, kl.endPointX * inv_width, kl.endPointY * inv_height, line_coords);
            //Record the grid's corresponding line
            for (const std::pair<int, int> &p : line_coords)
                grid.at(p.first, p.second).push_back(idx);
        }

        GridWindow w;
        w.width = std::make_pair(10, 0); // 10
        w.height = std::make_pair(0, 0);

        std::vector<int> matches_12;
        matchGrid(coords, mDescriptors_Line, grid, mDescriptorsRight_Line, directions, w, matches_12);

        for (unsigned int i1 = 0; i1 < matches_12.size(); ++i1) {
            const int i2 = matches_12[i1];
            if (i2 < 0) continue;

            // estimate the disparity of the endpoints
            Eigen::Vector3d sp_l; sp_l << mvKeys_Line[i1].startPointX, mvKeys_Line[i1].startPointY, 1.0;
            Eigen::Vector3d ep_l; ep_l << mvKeys_Line[i1].endPointX,   mvKeys_Line[i1].endPointY,   1.0;
            Eigen::Vector3d sp_r; sp_r << mvKeysRight_Line[i2].startPointX, mvKeysRight_Line[i2].startPointY, 1.0;
            Eigen::Vector3d ep_r; ep_r << mvKeysRight_Line[i2].endPointX,   mvKeysRight_Line[i2].endPointY,   1.0;

            double overlap = lineSegmentOverlapStereo( sp_l(1), ep_l(1), sp_r(1), ep_r(1) );

            double disp_s, disp_e;
            // the y coordinate would not change, so calculate the corresponding position in right image with the same y.
            sp_r << ( sp_r(0)*( sp_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - sp_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , sp_l(1) ,  1.0;
            ep_r << ( sp_r(0)*( ep_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - ep_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , ep_l(1) ,  1.0;
            filterLineSegmentDisparity( sp_l.head(2), ep_l.head(2), sp_r.head(2), ep_r.head(2), disp_s, disp_e );

            // check minimal disparity
            if( disp_s >= 1 && disp_e >= 1 // 1
                && std::abs( sp_l(1)-ep_l(1) ) > 0.1  // 0.1
                && std::abs( sp_r(1)-ep_r(1) ) > 0.1
                && overlap > 0.75 ){
                mvDisparity_l[i1] = make_pair(disp_s,disp_e);
                cv::Point3d p3D_s, p3D_e;
                float depth = mbf / disp_s;
                float u = mvKeys_Line[i1].startPointX;
                float v = mvKeys_Line[i1].startPointY;
                p3D_s.z = depth;
                p3D_s.x = (u - cx)*depth*invfx;
                p3D_s.y = (v - cy)*depth*invfy;

                depth = mbf / disp_e;
                u = mvKeys_Line[i1].endPointX;
                v = mvKeys_Line[i1].endPointY;
                p3D_e.z = depth;
                p3D_e.x = (u - cx)*depth*invfx;
                p3D_e.y = (v - cy)*depth*invfy;
                mvLineEndPoints[i1] = make_pair(p3D_s, p3D_e);
                cv::Point3d direction3D = p3D_e - p3D_s;
                float length3D = sqrt(direction3D.x*direction3D.x + direction3D.y*direction3D.y + direction3D.z*direction3D.z);
                direction3D = direction3D/length3D;
                mvLine3DDirections[i1] = direction3D;
            }
        }
    }

    void Frame::ComputePlansFromLines() {
        for(int i = 0; i < mvKeys_Line.size(); i++) {
            cv::Point3d directioni = mvLine3DDirections[i];
            if (directioni.x == 0 && directioni.y == 0 && directioni.z == 0)
                continue;
            for(int j=i+1; j < mvKeys_Line.size();j++) {
                cv::Point3d directionj = mvLine3DDirections[j];
                if (directionj.x == 0 && directionj.y == 0 && directionj.z == 0) {
                    continue;
                }
                float dot = abs(directioni.dot(directionj));
                if(dot > mfParallelAngle) {
                    if(mbUseParallelLines)
                        ComputePlansFromParallelLines(i,j);
                    else
                        continue;
                }else {
                    if (ConnectedLines(i, j)) {
                        cv::Point3d planeNormal = directioni.cross(directionj);
                        float norm = sqrt(planeNormal.x * planeNormal.x + planeNormal.y * planeNormal.y +
                                          planeNormal.z * planeNormal.z);
                        planeNormal.x = planeNormal.x / norm;
                        planeNormal.y = planeNormal.y / norm;
                        planeNormal.z = planeNormal.z / norm;

                        cv::Point3d p3Dis = mvLineEndPoints[i].first;
                        cv::Point3d p3Die = mvLineEndPoints[i].second;
                        cv::Point3d p3Djs = mvLineEndPoints[j].first;
                        cv::Point3d p3Dje = mvLineEndPoints[j].second;

                        float d1 = planeNormal.x * p3Dis.x + planeNormal.y * p3Dis.y + planeNormal.z * p3Dis.z;
                        float d2 = planeNormal.x * p3Die.x + planeNormal.y * p3Die.y + planeNormal.z * p3Die.z;
                        float d3 = planeNormal.x * p3Djs.x + planeNormal.y * p3Djs.y + planeNormal.z * p3Djs.z;
                        float d4 = planeNormal.x * p3Dje.x + planeNormal.y * p3Dje.y + planeNormal.z * p3Dje.z;
                        float dmin = 10000, dmax = -10000;
                        dmin = dmin < d1 ? dmin : d1;
                        dmin = dmin < d2 ? dmin : d2;
                        dmin = dmin < d3 ? dmin : d3;
                        dmin = dmin < d4 ? dmin : d4;
                        dmax = dmax > d1 ? dmax : d1;
                        dmax = dmax > d2 ? dmax : d2;
                        dmax = dmax > d3 ? dmax : d3;
                        dmax = dmax > d4 ? dmax : d4;

                        if (dmax - dmin > mfSamePlaneDis)
                            continue;

                        float planeDis = -(d1 + d2 + d3 + d4) / 4;

                        cv::Vec4f plane(planeNormal.x, planeNormal.y, planeNormal.z, planeDis);
                        if (plane[3] < 0) {
                            plane = -plane;
                            planeNormal = -planeNormal;
                        }
                        if (OldPlane(plane))
                            continue;
                        mvPlanes.push_back(plane);
                        mvPlaneNormal.push_back(planeNormal);
                        mvPlaneLineNo.push_back(make_pair(i, j));

                    }
                }
            }
        }
    }

    void Frame::ComputePlansFromParallelLines(int i, int j) {
        //Check position
        cv::Point2d lip1 = mvKeys_Line[i].getStartPoint();
        cv::Point2d lip2 = mvKeys_Line[i].getEndPoint();
        cv::Point2d ljp1 = mvKeys_Line[j].getStartPoint();
        cv::Point2d ljp2 = mvKeys_Line[j].getEndPoint();
        cv::Point2d lipc((lip1.x+lip2.x)/2, (lip1.y+lip2.y)/2);
        cv::Point2d ljpc((ljp1.x+ljp2.x)/2, (ljp1.y+ljp2.y)/2);
        float disi = sqrt(pow(lip1.x-lip2.x,2) + pow(lip1.y-lip2.y,2));
        float disj = sqrt(pow(ljp1.x-ljp2.x,2) + pow(ljp1.y-ljp2.y,2));
        float disij = 4 * sqrt(pow(lipc.x-ljpc.x,2) + pow(lipc.y-ljpc.y,2));
        if(disij > (disi + disj)){
            return;
        }

        cv::Point3d pi3D1 = mvLineEndPoints[i].first;
        cv::Point3d pi3D2 = mvLineEndPoints[i].second;
        cv::Point3d pj3D1 = mvLineEndPoints[j].first;
        cv::Point3d pj3D2 = mvLineEndPoints[j].second;
        cv::Point3d pi3Dc = (pi3D1 + pi3D2) / 2;
        cv::Point3d pj3Dc = (pj3D1 + pj3D2) / 2;
        float dis3Di = sqrt(pow(pi3D1.x-pi3D2.x,2)+pow(pi3D1.y-pi3D2.y,2)+pow(pi3D1.z-pi3D2.z,2));
        float dis3Dj = sqrt(pow(pj3D1.x-pj3D2.x,2)+pow(pj3D1.y-pj3D2.y,2)+pow(pj3D1.z-pj3D2.z,2));
        float dis3Dij = sqrt(pow(pi3Dc.x-pj3Dc.x,2)+pow(pi3Dc.y-pj3Dc.y,2)+pow(pi3Dc.z-pj3Dc.z,2));
        if(dis3Dij < 0.1)
            return;
        dis3Dij = 4*dis3Dij;
        if(dis3Dij > (dis3Di + dis3Dj))
            return;


        cv::Point3d directioni = mvLine3DDirections[i];
        cv::Point3d directionj = pj3Dc-pi3D1;
        float lengthj3D = sqrt(directionj.x*directionj.x + directionj.y*directionj.y + directionj.z*directionj.z);
        directionj = directionj/lengthj3D;

        cv::Point3d planeNormal = directioni.cross(directionj);
        float norm = sqrt(planeNormal.x*planeNormal.x + planeNormal.y*planeNormal.y + planeNormal.z*planeNormal.z);
        planeNormal.x = planeNormal.x / norm;
        planeNormal.y = planeNormal.y / norm;
        planeNormal.z = planeNormal.z / norm;

        cv::Point3d p3Dis = mvLineEndPoints[i].first;
        cv::Point3d p3Die = mvLineEndPoints[i].second;
        cv::Point3d p3Djs = mvLineEndPoints[j].first;
        cv::Point3d p3Dje = mvLineEndPoints[j].second;

        float d1 = planeNormal.x * p3Dis.x + planeNormal.y * p3Dis.y + planeNormal.z * p3Dis.z;
        float d2 = planeNormal.x * p3Die.x + planeNormal.y * p3Die.y + planeNormal.z * p3Die.z;
        float d3 = planeNormal.x * p3Djs.x + planeNormal.y * p3Djs.y + planeNormal.z * p3Djs.z;
        float d4 = planeNormal.x * p3Dje.x + planeNormal.y * p3Dje.y + planeNormal.z * p3Dje.z;
        float dmin=10000, dmax=-10000;
        dmin = dmin < d1 ? dmin : d1;
        dmin = dmin < d2 ? dmin : d2;
        dmin = dmin < d3 ? dmin : d3;
        dmin = dmin < d4 ? dmin : d4;
        dmax = dmax > d1 ? dmax : d1;
        dmax = dmax > d2 ? dmax : d2;
        dmax = dmax > d3 ? dmax : d3;
        dmax = dmax > d4 ? dmax : d4;

        if(dmax - dmin > mfSamePlaneDis)
            return;

        float planeDis = -(d1 + d2 + d3 + d4)/4;

        cv::Vec4f plane(planeNormal.x, planeNormal.y, planeNormal.z, planeDis);
        if(plane[3] < 0){
            plane = -plane;
            planeNormal = -planeNormal;
        }
        // Repeat Plane?
        for(auto pli : mvPlanes){
            float angle = plane[0]*pli[0] + plane[1]*pli[1] + plane[2]*pli[2];
            if(angle < mfAngleTh && angle > -mfAngleTh)
                continue;
            float disis = (pli[0] * p3Dis.x + pli[1] * p3Dis.y + pli[2] * p3Dis.z + pli[3]);
            float disie = (pli[0] * p3Die.x + pli[1] * p3Die.y + pli[2] * p3Die.z + pli[3]);
            float disjs = (pli[0] * p3Djs.x + pli[1] * p3Djs.y + pli[2] * p3Djs.z + pli[3]);
            float disje = (pli[0] * p3Dje.x + pli[1] * p3Dje.y + pli[2] * p3Dje.z + pli[3]);
            float dd = (disis + disie + disjs + disje)/4;
            if(dd > mfDisTh || dd < -mfDisTh)
                continue;
            return;
        }

//        if(OldPlane(plane))
//            return;

        mvPlanes.push_back(plane);
        mvPlaneNormal.push_back(planeNormal);
        mvPlaneLineNo.push_back(make_pair(i, j));
    }

    bool Frame::OldPlane(cv::Vec4f &pl) {
        for(auto pli : mvPlanes){
            float d = pl[3] - pli[3];
            float angle = pl[0]*pli[0] + pl[1]*pli[1] + pl[2]*pli[2];
            if(d > 0.2 || d < -0.2)
                continue;
            if(angle < 0.866 && angle > -0.866) //30 degrees
                continue;
            return true;
        }
        return false;
    }

    bool Frame::ConnectedLines(int i, int j) {
        cv::Point2d lip1 = mvKeys_Line[i].getStartPoint();
        cv::Point2d lip2 = mvKeys_Line[i].getEndPoint();
        cv::Point2d ljp1 = mvKeys_Line[j].getStartPoint();
        cv::Point2d ljp2 = mvKeys_Line[j].getEndPoint();

        cv::Point2d lipc((lip1.x+lip2.x)/2, (lip1.y+lip2.y)/2);
        cv::Point2d ljpc((ljp1.x+ljp2.x)/2, (ljp1.y+ljp2.y)/2);

        float disi = sqrt(pow(lip1.x-lip2.x,2) + pow(lip1.y-lip2.y,2));
        float disj = sqrt(pow(ljp1.x-ljp2.x,2) + pow(ljp1.y-ljp2.y,2));

        float disij = 2 * sqrt(pow(lipc.x-ljpc.x,2) + pow(lipc.y-ljpc.y,2));

        if(disij < (disi + disj)){
            cv::Point3d pi3D1 = mvLineEndPoints[i].first;
            cv::Point3d pi3D2 = mvLineEndPoints[i].second;
            cv::Point3d pj3D1 = mvLineEndPoints[j].first;
            cv::Point3d pj3D2 = mvLineEndPoints[j].second;

            cv::Point3d pi3Dc = (pi3D1 + pi3D2) / 2;
            cv::Point3d pj3Dc = (pj3D1 + pj3D2) / 2;

            float dis3Di = sqrt(pow(pi3D1.x-pi3D2.x,2)+pow(pi3D1.y-pi3D2.y,2)+pow(pi3D1.z-pi3D2.z,2));
            float dis3Dj = sqrt(pow(pj3D1.x-pj3D2.x,2)+pow(pj3D1.y-pj3D2.y,2)+pow(pj3D1.z-pj3D2.z,2));

            float dis3Dij = 2 * sqrt(pow(pi3Dc.x-pj3Dc.x,2)+pow(pi3Dc.y-pj3Dc.y,2)+pow(pi3Dc.z-pj3Dc.z,2));

            if(dis3Dij < (dis3Di + dis3Dj))
                return true;
        }

        return false;


    }

    void Frame::filterLineSegmentDisparity( Eigen::Vector2d spl, Eigen::Vector2d epl, Eigen::Vector2d spr,
                                            Eigen::Vector2d epr, double &disp_s, double &disp_e ) {
        disp_s = spl(0) - spr(0);
        disp_e = epl(0) - epr(0);
        // if they are too different, ignore them
        if(  min( disp_s, disp_e ) / max( disp_s, disp_e ) < 0.7){
            disp_s = -1.0;
            disp_e = -1.0;
        }
    }


void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;

    mTwc = cv::Mat::eye(4,4,mTcw.type());
    mRwc.copyTo(mTwc.rowRange(0,3).colRange(0,3));
    mOw.copyTo(mTwc.rowRange(0,3).col(3));
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}
    pair<cv::Mat,cv::Mat> Frame::UnprojectEndPoints(const int &i){
        cv::Point3d pi = mvLineEndPoints[i].first;
        cv::Point3d pj = mvLineEndPoints[i].second;
        cv::Mat x3Di = (cv::Mat_<float>(3,1) << pi.x, pi.y, pi.z);
        cv::Mat x3Dj = (cv::Mat_<float>(3,1) << pj.x, pj.y, pj.z);
        return make_pair(mRwc*x3Di+mOw,mRwc*x3Dj+mOw);
    }

    cv::Mat Frame::ComputeWorldPlane(int i) {
        cv::Vec4f pl = mvPlanes[i];
        cv::Mat x3Dc = (cv::Mat_<float>(4,1) << pl[0], pl[1], pl[2], pl[3]);
        cv::Mat temp;
        cv::transpose(mTcw, temp);
        return temp*x3Dc;
    }

    cv::Mat Frame::ComputeCameraPlane(int i) {
        cv::Vec4f pl = mvPlanes[i];
        cv::Mat x3Dc = (cv::Mat_<float>(4,1) << pl[0], pl[1], pl[2], pl[3]);
        return x3Dc;
    }
} //namespace ORB_SLAM
