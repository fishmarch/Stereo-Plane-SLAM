//
// Created by fishmarch on 20-6-20.
//

#include "GridStructure.h"

//STL
#include <algorithm>
#include <stdexcept>



namespace ORB_SLAM2 {

    void getLineCoords(double x1, double y1, double x2, double y2, std::list<std::pair<int, int>> &line_coords) {
        line_coords.clear();

        LineIterator it(x1, y1, x2, y2);

        std::pair<int, int> p;
        while (it.getNext(p))
            line_coords.push_back(p);
    }

    GridStructure::GridStructure(int rows, int cols)
            : rows(rows), cols(cols) {

        if (rows <= 0 || cols <= 0)
            throw std::runtime_error("[GridStructure] invalid dimension");

        grid.resize(cols, std::vector<std::list<int>>(rows));
    }

    GridStructure::~GridStructure() {

    }

    std::list<int>& GridStructure::at(int x, int y) {

        if (x >= 0 && x < cols &&
            y >= 0 && y < rows)
            return grid[x][y];
        else
            return out_of_bounds;
    }

    void GridStructure::get(int x, int y, const GridWindow &w, std::unordered_set<int> &indices) const {

        int min_x = std::max(0, x - w.width.first);
        int max_x = std::min(cols, x + w.width.second + 1);

        int min_y = std::max(0, y - w.height.first);
        int max_y = std::min(rows, y + w.height.second + 1);

        for (int x_ = min_x; x_ < max_x; ++x_)
            for (int y_ = min_y; y_ < max_y; ++y_)
                indices.insert(grid[x_][y_].begin(), grid[x_][y_].end());
    }

    void GridStructure::clear() {

        for (int x = 0; x < cols; ++x)
            for (int y = 0; y < rows; ++y)
                grid[x][y].clear();
    }

    LineIterator::LineIterator(const double x1_, const double y1_, const double x2_, const double y2_)
            : x1(x1_), y1(y1_), x2(x2_), y2(y2_), steep(std::abs(y2_ - y1_) > std::abs(x2_ - x1_)) {

        if (steep) {
            std::swap(x1, y1);
            std::swap(x2, y2);
        }

        if(x1 > x2) {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }

        dx = x2 - x1;
        dy = std::abs(y2 - y1);

        error = dx / 2.0;
        ystep = (y1 < y2) ? 1 : -1;

        x = static_cast<int>(x1);
        y = static_cast<int>(y1);

        maxX = static_cast<int>(x2);
    }

    bool LineIterator::getNext(std::pair<int, int> &pixel) {

        if (x > maxX)
            return false;

        if (steep)
            pixel = std::make_pair(y, x);
        else
            pixel = std::make_pair(x, y);

        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }

        x++;
        return true;
    }
} //namesapce ORB_SLAM2
