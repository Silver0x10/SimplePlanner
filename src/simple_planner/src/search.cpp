#include "search.hpp"

using namespace std;

namespace planner {

    Node::Node(int row, int col, int cost, int vector_index, Node* parent) : row(row), col(col), cost(cost), vector_index(vector_index), parent(parent) {}

    bool Node::equals(const Node& other) const {
        return this->row == other.row and this->col == other.col; // and this->cost == other.cost;
    }

    bool in_bounds(cv::Mat map, int row, int col) {
        return row >= 0 and row < map.rows and col >= 0 && col < map.cols;
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
            }
        }
        map_image.at<cv::Vec3b>(goal.row, goal.col) = cv::Vec3b(255, 0, 0);
        Node* current = goal.parent;
        cout << "start" << endl;
        while (current != nullptr) {
            cout << current->cost << "\t" << current->row << "\t" << current->col << endl;
            map_image.at<cv::Vec3b>(current->row, current->col) = cv::Vec3b(0, 195, 255);
            current = current->parent;
        }
        cout << "stop" << endl;
        map_image.at<cv::Vec3b>(root.row, root.col) = cv::Vec3b(0, 0, 255);

        cv::imwrite("visited.png", map_image);
    }


    Node search(Node root, Node goal, cv::Mat map) {
        std::vector<std::pair<int, int>> movements = { {-1, 0}, {1, 0},  {0, -1}, {0, 1} };
        queue<Node*> frontier;
        vector<vector<bool>> visited(map.rows, vector<bool>(map.cols, false));
        frontier.push(&root);
        visited[root.row][root.col] = true;
        while(!frontier.empty()) {
            Node* current = frontier.front();
            frontier.pop();
            if(current->equals(goal)) {
                save_explored_nodes(map, visited, root, *current);
                return *current;
            }
            for(const auto& move: movements) {
                int n_row = current->row + move.first;
                int n_col = current->col + move.second;
                if( in_bounds(map, n_row, n_col) and map.at<int>(n_row, n_col) != 0) {
                    if( not visited[n_row][n_col] ) { 
                        int vector_index = (map.rows - n_row - 1) * map.cols + n_col;
                        Node* child = new Node(n_row, n_col, current->cost+1, vector_index, current);
                        frontier.push(child);
                        visited[n_row][n_col] = true;
                    }
                }
            }
        }
        save_explored_nodes(map, visited, root, goal);
        return root; // path not found
    }     

}
