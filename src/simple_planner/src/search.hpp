#include <vector>
#include <opencv2/opencv.hpp>
#include <queue>
#include <set>

using namespace std;

namespace planner {

    class Node {
        public:
            int row;
            int col;
            int cost;
            int vector_index;
            Node* parent;

            Node(int row, int col, int cost, int vector_index, Node* parent = nullptr);

            bool equals(const Node& other) const;
    };

    bool in_bounds(cv::Mat map, int row, int col);

    Node search(Node root, Node goal, cv::Mat map);

}