#include <vector>
#include <opencv2/opencv.hpp>
#include <queue>

#include "utils.hpp"

using namespace std;

namespace planner {

    class Node {
        public:
            int row;
            int col;
            int steps;
            int vector_index;
            int closest_object_distance;
            Node* parent;

            Node();
            Node(int row, int col, int steps, int vector_index, int closest_object_distance, Node* parent = nullptr);

            bool equals(const Node& other) const;
    };

    struct NodeComparator {
        bool operator()(Node* node_1, Node* node_2);
    };

    Node search(Node& root, Node& goal, cv::Mat distance_map);

}