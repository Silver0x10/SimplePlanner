#include <cstdio>
#include <chrono>
#include <stdio.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <opencv2/opencv.hpp>

#include "search.hpp"

using namespace std;
using namespace std::chrono_literals;

class SimplePlanner : public rclcpp::Node {
  
    private:
    
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
        geometry_msgs::msg::PoseStamped goal_;

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
        nav_msgs::msg::OccupancyGrid map_;
        cv::Mat distance_map_;
        
        // pose stuff 
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        geometry_msgs::msg::TransformStamped robot_pose_;

        planner::Node root_node;
        planner::Node reached_node_;

        // // Map pulisher (Not used)
        // rclcpp::TimerBase::SharedPtr timer_;
        // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

        // Path publisher
        rclcpp::TimerBase::SharedPtr path_timer_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;



        void goal_received_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            printf("goal received\n");
            goal_.header = msg->header;
            goal_.pose = msg->pose;
            RCLCPP_INFO(this->get_logger(), "I heard the goal pose: '%f %f %f'", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

            std::string source_frame = "map";
            std::string target_frame = "base_link";
            
            int attempt = 0;
            while(!tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero)) {
                attempt++;
                RCLCPP_WARN(this->get_logger(), "Transform from '%s' to '%s' is not available yet. Attempt %i", source_frame.c_str(), target_frame.c_str(), attempt);
                if(attempt >= 10) { 
                    RCLCPP_ERROR(this->get_logger(), "Robot pose not available");
                    return;
                }
                sleep(1);
            }
            robot_pose_ = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero);
            RCLCPP_INFO(this->get_logger(), "I heard the robot pose: '%f %f %f'", robot_pose_.transform.translation.x, robot_pose_.transform.translation.y, robot_pose_.transform.translation.z);

            int robot_col = int( robot_pose_.transform.translation.x / map_.info.resolution );
            int robot_row = map_.info.height - int( robot_pose_.transform.translation.y / map_.info.resolution ) - 1;
            int robot_vector_index = (map_.info.height - robot_row - 1) * map_.info.width + robot_col;
            int robot_closest_object_distance = distance_map_.at<int>(robot_row, robot_col);
            root_node = planner::Node(robot_row, robot_col, 0, robot_vector_index, robot_closest_object_distance, nullptr);

            int goal_col = int( goal_.pose.position.x / map_.info.resolution );
            int goal_row = map_.info.height - int( goal_.pose.position.y / map_.info.resolution ) - 1;
            int goal_vector_index = (map_.info.height - goal_row - 1) * map_.info.width + goal_col;
            int goal_closest_object_distance = distance_map_.at<int>(goal_row, goal_col);
            planner::Node goal_node(goal_row, goal_col, -1, goal_vector_index, goal_closest_object_distance);

            reached_node_ = planner::search(root_node, goal_node, distance_map_);
            cout << "start row: " << root_node.row << "\tcol: " << root_node.col << endl;
            cout << "reached row: " << reached_node_.row << "\tcol: " << reached_node_.col << endl;
            cout << "desired row: " << goal_node.row << "\tcol: " << goal_node.col << endl;

            save_map();
        }

        // TODO convert as a node service
        void save_map() { 
            cv::Mat map_image(map_.info.height, map_.info.width, CV_8UC3, cv::Scalar(128, 128, 128));
            
            for(unsigned int col = 0; col < map_.info.width; col++) {
                for(unsigned int row = 0; row < map_.info.height; row++) {
                    int i = (map_.info.height - row - 1) * map_.info.width + col; // row_major starting from the bottom left corner
                    if(map_.data[i] == 0) { // free space
                        map_image.at<cv::Vec3b>(row, col) = cv::Vec3b(255, 255, 255);
                    } else 
                    if (map_.data[i] == 100) { // occupied space
                        map_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
                    }
                }
            }

            int robot_col = int( robot_pose_.transform.translation.x / map_.info.resolution );
            int robot_row = map_.info.height - int( robot_pose_.transform.translation.y / map_.info.resolution ) - 1;
            for(int d_col = -1; d_col <= 1; d_col++) {
                for(int d_row = -1; d_row <= 1; d_row++) {
                    int n_row = robot_row + d_row;
                    int n_col = robot_col + d_col;
                    map_image.at<cv::Vec3b>(n_row, n_col) = cv::Vec3b(255, 0, 0);
                }
            }

            cv::imwrite("occupancy_grid.png", map_image);
        }

        void map_received_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            map_.info = msg->info;
            map_.data = msg->data;
            distance_map_ = planner::compute_costmap(map_);
            RCLCPP_INFO(this->get_logger(), "I heard the map: w:%d h:%d resolution:%f'", msg->info.width, msg->info.height, msg->info.resolution);
        }

        // // Map pulisher (Not used)
        // void timer_callback() {
        //     publisher_->publish(map_);
        //     RCLCPP_INFO(this->get_logger(), "map: '%d %d %f'", map_.info.width, map_.info.height, map_.info.resolution);
        // }

        void path_timer_callback() {
            // auto msg = nav_msgs::msg::Path();
            nav_msgs::msg::Path msg;
            msg.header.stamp = this->now(); // see ros::Time::Now()
            msg.header.frame_id = "map";

            vector<planner::Node*> path_nodes;
            path_nodes.push_back(&reached_node_);
            planner::Node* current = reached_node_.parent;
            while(current != nullptr) {
                path_nodes.push_back(current);
                current = current->parent;
            }
            
            int path_len = path_nodes.size();
            // RCLCPP_INFO(this->get_logger(), "path len: %d", path_len);
            
            // // for(auto i=path_nodes.rbegin(); i!=path_nodes.rend(); ++i) {
            for(auto i=0; i<path_len; ++i) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = this->now();
                pose_msg.header.frame_id = "base_link";
                pose_msg.pose.position.x = path_nodes[path_len-1 - i]->col * map_.info.resolution;
                pose_msg.pose.position.y = (map_.info.height - 1 - path_nodes[path_len-1 - i]->row ) * map_.info.resolution;
                msg.poses.push_back(pose_msg);
            }
            path_publisher_->publish(msg);
        }

    public:

        SimplePlanner() : rclcpp::Node("simple_planner") {
            rclcpp::QoS qos(rclcpp::KeepLast(10));
            
            sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", qos.transient_local(), std::bind(&SimplePlanner::map_received_callback, this, std::placeholders::_1));

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            sub_goal_ =  this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", qos.durability_volatile(), std::bind(&SimplePlanner::goal_received_callback, this, std::placeholders::_1));

            // // Map pulisher (Not used)
            // publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("my_map", qos);
            // timer_ = this->create_wall_timer(500ms, std::bind(&SimplePlanner::timer_callback, this));

            path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", qos.transient_local());
            path_timer_ = this->create_wall_timer(200ms, std::bind(&SimplePlanner::path_timer_callback, this));
        }
        
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePlanner>());
    rclcpp::shutdown();
    return 0;
}
