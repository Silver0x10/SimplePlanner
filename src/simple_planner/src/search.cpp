#include "search.hpp"

using namespace std;

namespace planner {

    Node::Node(int row, int col, int steps, int vector_index, int closest_object_distance, Node* parent) : row(row), col(col), steps(steps), vector_index(vector_index), closest_object_distance(closest_object_distance), parent(parent) {}

    bool Node::equals(const Node& other) const {
        return this->row == other.row and this->col == other.col; // and this->steps == other.steps;
    }

    bool NodeComparator::operator()(Node* node_1, Node* node_2) {
        if(node_1->closest_object_distance != node_2->closest_object_distance) return node_1->closest_object_distance < node_2->closest_object_distance;
        return node_1->steps > node_2->steps;
    }


    bool in_bounds(cv::Mat map, int row, int col) {
        return row >= 0 and row < map.rows and col >= 0 && col < map.cols;
    }


    cv::Mat compute_costmap(const nav_msgs::msg::OccupancyGrid& map) { // simple BFS starting from obstacles
        cv::Mat distance_map(map.info.height, map.info.width, CV_32S, cv::Scalar(255));
        queue<pair<int, int>*> frontier;
        vector<vector<bool>> visited(int(map.info.height), vector<bool>(int(map.info.width), false));
        for(unsigned int col = 0; col < map.info.width; col++) {
            for(unsigned int row = 0; row < map.info.height; row++) {
                int i = (map.info.height - row - 1) * map.info.width + col; // row_major starting from the bottom left corner
                if(map.data[i] == 100) {
                    distance_map.at<int>(row, col) = 0;
                    pair<int, int>* obstacle_coords = new pair<int, int>(row, col);
                    frontier.push(obstacle_coords);
                    visited[row][col] = true;
                } else if(map.data[i] == -1){
                    distance_map.at<int>(row, col) = 0;
                    visited[row][col] = true;
                }

                // if(map.data[i] == 0) { // free space
                //     distance_map.at<int>(row, col) = 1;
                // }
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

        return distance_map;
    }


    void save_explored_nodes(cv::Mat map, vector<vector<bool>> visited, Node root, Node goal) { 
        cv::Mat map_image(map.rows, map.cols, CV_8UC3, cv::Scalar(255, 255, 255)); // BGR
        
        for(unsigned int col = 0; col < uint(map.cols); col++) {
            for(unsigned int row = 0; row < uint(map.rows); row++) {
                // int i = (map.rows - row - 1) * map.cols + col; // row_major starting from the bottom left corner
                if(map.at<int>(row,col) == 0) {
                    map_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
                } else if(visited[row][col]) {
                    map_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 255, 0);
                }
                // TODO save in the distance_map init
                // to show the costmap
                // map_image.at<cv::Vec3b>(row, col) = cv::Vec3b(map.at<int>(row,col)*5, map.at<int>(row,col)*5, 0);
            }
        }
        map_image.at<cv::Vec3b>(goal.row, goal.col) = cv::Vec3b(255, 0, 0);
        Node* current = goal.parent;
        cout << "start" << endl;
        while (current != nullptr) {
            // cout << current->steps << "\t" << current->row << "\t" << current->col << endl;
            map_image.at<cv::Vec3b>(current->row, current->col) = cv::Vec3b(0, 195, 255);
            current = current->parent;
        }
        cout << "stop" << endl;
        map_image.at<cv::Vec3b>(root.row, root.col) = cv::Vec3b(0, 0, 255);

        cv::imwrite("visited.png", map_image);
    }


    Node search(Node root, Node goal, cv::Mat distance_map) {
        vector<pair<int, int>> movements = { {-1, 0}, {1, 0},  {0, -1}, {0, 1} };
        priority_queue<Node*, vector<Node*>, NodeComparator> frontier;
        vector<vector<bool>> visited(distance_map.rows, vector<bool>(distance_map.cols, false));
        frontier.push(&root);
        visited[root.row][root.col] = true;
        while(!frontier.empty()) {
            Node* current = frontier.top();
            frontier.pop();
            if(current->equals(goal)) {
                save_explored_nodes(distance_map, visited, root, *current);
                return *current;
            }
            for(const auto& move: movements) {
                int n_row = current->row + move.first;
                int n_col = current->col + move.second;
                if( in_bounds(distance_map, n_row, n_col) and distance_map.at<int>(n_row, n_col) != 0) {
                    if( not visited[n_row][n_col] ) { // check om the cost not included since the distance from the nearest obstable (which we use as cost) doesn't change
                        int vector_index = (distance_map.rows - n_row - 1) * distance_map.cols + n_col;
                        int closest_object_distance = distance_map.at<int>(n_row, n_col);
                        Node* child = new Node(n_row, n_col, current->steps+1, vector_index, closest_object_distance, current);
                        frontier.push(child);
                        visited[n_row][n_col] = true;
                    }
                }
            }
        }
        save_explored_nodes(distance_map, visited, root, goal);
        return root; // path not found
    }     

}
