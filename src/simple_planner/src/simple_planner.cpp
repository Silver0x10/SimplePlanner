#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class SimplePlanner : public rclcpp::Node {
  
    private:
    
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map;
        
        // pose stuff 
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
        void goal_received_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            printf("goal received\n");
            RCLCPP_INFO(this->get_logger(), "I heard the goal pose: '%f %f %f'", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

            std::string source_frame = "map";
            std::string target_frame = "base_link";
            
            geometry_msgs::msg::TransformStamped t;
            t = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            RCLCPP_INFO(this->get_logger(), "I heard the robot pose: '%f %f %f'", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
        }

        void map_received_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            printf("map received\n");
            RCLCPP_INFO(this->get_logger(), "I heard the map: '%d %d %f'", msg->info.width, msg->info.height, msg->info.resolution);
        }

    public:

        SimplePlanner() : Node("simple_planner") {
            rclcpp::QoS qos(rclcpp::KeepLast(10));
            qos.transient_local();
            map = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", qos, std::bind(&SimplePlanner::map_received_callback, this, std::placeholders::_1));

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            sub_goal =  this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(&SimplePlanner::goal_received_callback, this, std::placeholders::_1));
        }
        
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePlanner>());
    rclcpp::shutdown();
    return 0;
}
