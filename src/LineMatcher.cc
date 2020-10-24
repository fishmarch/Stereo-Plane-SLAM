//
// Created by fishmarch on 20-6-20.
//

#include "LineMatcher.h"

//STL
#include <cmath>
#include <functional>
#include <future>
#include <limits>
#include <stdexcept>

//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "GridStructure.h"


namespace ORB_SLAM2 {



    int distance(const cv::Mat &a, const cv::Mat &b) {

        // adapted from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist = 0;
        for(int i = 0; i < 8; i++, pa++, pb++) {
            unsigned  int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

    int matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1,
                  const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2,
                  const GridWindow &w,
                  std::vector<int> &matches_12) {

        if (lines1.size() != desc1.rows)
            throw std::runtime_error("[matchGrid] Each line needs a corresponding descriptor!");

        int matches = 0;
        matches_12.resize(desc1.rows, -1);

        int best_d, best_d2, best_idx;
        std::vector<int> matches_21, distances;

        if (true) { // true : double-checking the matches between the two images
            matches_21.resize(desc2.rows, -1);
            distances.resize(desc2.rows, std::numeric_limits<int>::max());
        }

        for (int i1 = 0, nsize = lines1.size(); i1 < nsize; ++i1) {

            best_d = std::numeric_limits<int>::max();
            best_d2 = std::numeric_limits<int>::max();
            best_idx = -1;

            const line_2d &coords = lines1[i1];
            cv::Mat desc = desc1.row(i1);

            const point_2d sp = coords.first;
            const point_2d ep = coords.second;

            std::pair<double, double> v = std::make_pair(ep.first - sp.first, ep.second - sp.second);
            normalize(v);

            std::unordered_set<int> candidates;
            grid.get(sp.first, sp.second, w, candidates);
            grid.get(ep.first, ep.second, w, candidates);

            if (candidates.empty()) continue;
            for (const int &i2 : candidates) {
                if (i2 < 0 || i2 >= desc2.rows) continue;

                if (std::abs(dot(v, directions2[i2])) < 0.75) //0.75  almost 40 degrees  0.94 20 degrees
                    continue;

                const int d = distance(desc, desc2.row(i2));

                if (true) {
                    if (d < distances[i2]) {
                        distances[i2] = d;
                        matches_21[i2] = i1;
                    } else continue;
                }

                if (d < best_d) {
                    best_d2 = best_d;
                    best_d = d;
                    best_idx = i2;
                } else if (d < best_d2)
                    best_d2 = d;
            }

            if (best_d < best_d2 * 0.75) {  // 0.9
                matches_12[i1] = best_idx;
                matches++;
            }
        }

        if (true) {
            for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
                int &i2 = matches_12[i1];
                if (i2 >= 0 && matches_21[i2] != i1) {
                    i2 = -1;
                    matches--;
                }
            }
        }

        return matches;
    }

    int matchGrid(const std::vector<line_2d> &lines1, const std::vector<cv::Vec4f> &lps1, const GridStructure &grid,
                  const std::vector<cv::Vec4f> &lps2, const std::vector<std::pair<double, double>> &directions2,
                  const GridWindow &w, std::vector<int> &matches_12) {


        int matches = 0;
        matches_12.resize(lps1.size(), -1);

        int best_d, best_d2, best_idx;
        std::vector<int> matches_21, distances;

        if (true) { // true : double-checking the matches between the two images
            matches_21.resize(lps2.size(), -1);
            distances.resize(lps2.size(), std::numeric_limits<int>::max());
        }

        for (int i1 = 0, nsize = lines1.size(); i1 < nsize; ++i1) {
            best_d = std::numeric_limits<int>::max();
            best_d2 = std::numeric_limits<int>::max();
            best_idx = -1;
            const line_2d &coords = lines1[i1];
            const point_2d sp = coords.first;
            const point_2d ep = coords.second;

            std::unordered_set<int> candidates;
            grid.get(sp.first, sp.second, w, candidates);
            grid.get(ep.first, ep.second, w, candidates);
            if (candidates.empty()) continue;
            cv::Vec4f lpi1 = lps1[i1];
            std::pair<double, double> v1 = std::make_pair(lpi1[2] - lpi1[0], lpi1[3] - lpi1[1]);
            normalize(v1);
            for (const int &i2 : candidates) {
                if (i2 < 0 || i2 >= lps2.size()) continue;

                cv::Vec4f lpi2 = lps1[i2];
                std::pair<double, double> v2 = std::make_pair(lpi2[2] - lpi2[0], lpi2[3] - lpi2[1]);
                normalize(v2);

                if (std::abs(dot(v1, v2)) < 0.996) //0.996 5 deg  0.94 20 deg
                    continue;

            }

        }

        if (true) {
            for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
                int &i2 = matches_12[i1];
                if (i2 >= 0 && matches_21[i2] != i1) {
                    i2 = -1;
                    matches--;
                }
            }
        }

        return matches;
    }

    double lineSegmentOverlapStereo(double spl_obs, double epl_obs, double spl_proj, double epl_proj)
    {
        double overlap = 1.f;

        if( fabs( epl_obs - spl_obs ) > 0.1 ){ // normal lines (verticals included) //0.1
            double sln    = min(spl_obs,  epl_obs);
            double eln    = max(spl_obs,  epl_obs);
            double spn    = min(spl_proj, epl_proj);
            double epn    = max(spl_proj, epl_proj);

            double length = eln-spn;

            if ( (epn < sln) || (spn > eln) )
                overlap = 0.f;
            else{
                if ( (epn>eln) && (spn<sln) )
                    overlap = eln-sln;
                else
                    overlap = min(eln,epn) - max(sln,spn);
            }
            if(length>0.01f)
                overlap = overlap / length;
            else
                overlap = 0.f;

            if( overlap > 1.f )
                overlap = 1.f;
        }
        return overlap;
    }
} //namesapce ORB_SLAM2
