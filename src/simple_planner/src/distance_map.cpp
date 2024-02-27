#include "distance_map.hpp"

namespace planner {

    void save_distance_map(cv::Mat distance_map) {
        cv::Mat distance_map_image(distance_map.rows, distance_map.cols, CV_8UC3, cv::Scalar(0, 0, 0)); // BGR

        for(auto i=0; i < distance_map.rows; i++) {
            for(auto j=0; j < distance_map.cols; j++) {
                distance_map_image.at<cv::Vec3b>(i, j) = cv::Vec3b(distance_map.at<int>(i, j)*5, distance_map.at<int>(i, j)*5, 0);
            }
        }
        cv::imwrite("distance_map.png", distance_map_image);
    }

    cv::Mat compute_distance_map(const nav_msgs::msg::OccupancyGrid& map) { // simple BFS starting from obstacles
        cv::Mat distance_map(map.info.height, map.info.width, CV_32S, cv::Scalar(255));
        queue<pair<int, int>*> frontier;
        vector<vector<bool>> visited(int(map.info.height), vector<bool>(int(map.info.width), false));
        for(unsigned int col = 0; col < map.info.width; col++) {
            for(unsigned int row = 0; row < map.info.height; row++) {
                int i = (map.info.height - row - 1) * map.info.width + col; // map is row_major ordered starting from the bottom left corner
                if(map.data[i] == 100) { // occupied cells
                    distance_map.at<int>(row, col) = 0;
                    pair<int, int>* obstacle_coords = new pair<int, int>(row, col);
                    frontier.push(obstacle_coords);
                    visited[row][col] = true;
                } else if(map.data[i] == -1) { // unknown cells
                    distance_map.at<int>(row, col) = 0;
                    visited[row][col] = true;
                }
            }
        }

        while(!frontier.empty()) {
            pair<int, int>* current = frontier.front();
            frontier.pop();
            for(int d_col = -1; d_col <= 1; d_col++) {
                for(int d_row = -1; d_row <= 1; d_row++) {
                    if(d_col == 0 and d_row ==0 ) continue;
                    int n_row = current->first + d_row;
                    int n_col = current->second + d_col;
                    if( in_bounds(distance_map, n_row, n_col) and not visited[n_row][n_col] ) { 
                        pair<int, int>* child = new pair<int, int>(n_row, n_col);
                        frontier.push(child);
                        visited[n_row][n_col] = true;
                        distance_map.at<int>(n_row, n_col) = distance_map.at<int>(current->first, current->second) + 1;
                    }
                }
            }
        }

        save_distance_map(distance_map);

        return distance_map;
    }

}