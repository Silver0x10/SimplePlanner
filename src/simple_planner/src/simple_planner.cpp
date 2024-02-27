#include <chrono>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "search.hpp"
#include "distance_map.hpp"

using namespace std;
using namespace std::chrono_literals;

class SimplePlanner : public rclcpp::Node {
  
    private:
    
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
        geometry_msgs::msg::PoseStamped goal_;

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
        nav_msgs::msg::OccupancyGrid map_;
        cv::Mat distance_map_;
        
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        geometry_msgs::msg::TransformStamped robot_pose_;

        planner::Node root_node;
        planner::Node reached_node_;

        rclcpp::TimerBase::SharedPtr path_timer_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
        nav_msgs::msg::Path path_;

        void goal_received_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            goal_.header = msg->header;
            goal_.pose = msg->pose;
            RCLCPP_INFO(this->get_logger(), "I heard the goal pose: '%f %f %f'", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

            string source_frame = "map";
            string target_frame = "base_link";
            
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

            RCLCPP_INFO(this->get_logger(), "START map coords --> row: %d\t col: %d", root_node.row, root_node.col);
            RCLCPP_INFO(this->get_logger(), "GOAL  map coords --> row: %d\t col: %d", goal_node.row, goal_node.col);
            reached_node_ = planner::search(root_node, goal_node, distance_map_);
            if(reached_node_.equals(goal_node)) RCLCPP_INFO(this->get_logger(), "Path computed :)");
            else RCLCPP_INFO(this->get_logger(), "The goal cannot be reached :(");

            path_.header.frame_id = "map";
            path_.poses.clear();
            vector<planner::Node*> path_nodes;
            path_nodes.push_back(&reached_node_);
            planner::Node* current = reached_node_.parent;
            while(current != nullptr) {
                path_nodes.push_back(current);
                current = current->parent;
            }
            for(auto i=path_nodes.rbegin(); i!=path_nodes.rend(); ++i) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = this->now();
                pose_msg.header.frame_id = "base_link";
                auto& node = *i;
                pose_msg.pose.position.x = node->col * map_.info.resolution;
                pose_msg.pose.position.y = (map_.info.height - 1 - node->row ) * map_.info.resolution;
                path_.poses.push_back(pose_msg);
            }
        }

        void map_received_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            map_.header = msg->header;
            map_.info = msg->info;
            map_.data = msg->data; // data is row_major ordered starting from the bottom left corner
            distance_map_ = planner::compute_distance_map(map_);
            RCLCPP_INFO(this->get_logger(), "I heard the map: w:%d h:%d resolution:%f'", msg->info.width, msg->info.height, msg->info.resolution);
        }

        void path_timer_callback() {
            path_.header.stamp = this->now();
            path_publisher_->publish(path_);
        }

    public:

        SimplePlanner() : rclcpp::Node("simple_planner") {
            rclcpp::QoS qos(rclcpp::KeepLast(10));
            
            sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", qos.transient_local(), std::bind(&SimplePlanner::map_received_callback, this, std::placeholders::_1));

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            sub_goal_ =  this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", qos.durability_volatile(), std::bind(&SimplePlanner::goal_received_callback, this, std::placeholders::_1));

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
