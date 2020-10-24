//
// Created by fishmarch on 20-6-20.
//

#ifndef ORB_SLAM2_LINEMATCHER_H
#define ORB_SLAM2_LINEMATCHER_H


//STL
#include <utility>
#include <vector>

//OpenCV
#include <opencv2/core.hpp>

#include "GridStructure.h"
#include "Frame.h"
#include "MapPoint.h"


namespace ORB_SLAM2 {

    class Frame;
    class MapPoint;
    class MapLine;

    typedef std::pair<int, int> point_2d;
    typedef std::pair<point_2d, point_2d> line_2d;

    inline double dot(const std::pair<double, double> &a, const std::pair<double, double> &b) {
        return (a.first*b.first + a.second*b.second);
    }

    inline void normalize(std::pair<double, double> &v) {
        double magnitude = std::sqrt(dot(v, v));

        v.first /= magnitude;
        v.second /= magnitude;
    }

    int matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

    int match(const std::vector<MapLine*> &mvpLocalMapLines, Frame &CurrentFrame, float nnr, std::vector<int> &matches_12);

    int match(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

    int distance(const cv::Mat &a, const cv::Mat &b);

//Points
    int matchGrid(const std::vector<point_2d> &points1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const GridWindow &w, std::vector<int> &matches_12);

//Lines
    int matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2, const GridWindow &w, std::vector<int> &matches_12);

    int matchGrid(const std::vector<line_2d> &lines1, const std::vector<cv::Vec4f> &lps1, const GridStructure &grid, const std::vector<cv::Vec4f> &lps2, const std::vector<std::pair<double, double>> &directions2, const GridWindow &w, std::vector<int> &matches_12);

    double lineSegmentOverlapStereo(double spl_obs, double epl_obs, double spl_proj, double epl_proj);

} // namesapce ORB_SLAM2

#endif //ORB_SLAM2_LINEMATCHER_H
