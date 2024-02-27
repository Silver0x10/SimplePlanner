#include "utils.hpp"

namespace planner {

    bool in_bounds(cv::Mat map, int row, int col) {
        return row >= 0 and row < map.rows and col >= 0 && col < map.cols;
    }

}