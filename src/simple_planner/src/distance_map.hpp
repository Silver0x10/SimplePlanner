#include <vector>
#include <opencv2/opencv.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "utils.hpp"

using namespace std;

namespace planner {

    cv::Mat compute_distance_map(const nav_msgs::msg::OccupancyGrid& map);

}