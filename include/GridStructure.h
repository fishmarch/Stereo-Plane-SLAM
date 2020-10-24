//
// Created by fishmarch on 20-6-20.
//

#ifndef ORB_SLAM2_GRIDSTRUCTURE_H
#define ORB_SLAM2_GRIDSTRUCTURE_H


//STL
#include <list>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ORB_SLAM2 {

    void getLineCoords(double x1, double y1, double x2, double y2, std::list<std::pair<int, int>> &line_coords);

    struct GridWindow {
        std::pair<int, int> width, height;
    };

    class GridStructure {
    public:

        int rows, cols;

        GridStructure(int rows, int cols);

        ~GridStructure();

        std::list<int>& at(int x, int y);

        void get(int x, int y, const GridWindow &w, std::unordered_set<int> &indices) const;

        void clear();

    private:

        std::vector<std::vector<std::list<int>>> grid;
        std::list<int> out_of_bounds;
    };

    class LineIterator {
    public:

        LineIterator(const double x1_, const double y1_, const double x2_, const double y2_);

        bool getNext(std::pair<int, int> &pixel);

    private:

        const bool steep;
        double x1, y1, x2, y2;

        double dx, dy, error;

        int maxX, ystep;
        int y, x;
    };


} // namespace ORB_SLAM2

#endif //ORB_SLAM2_GRIDSTRUCTURE_H
