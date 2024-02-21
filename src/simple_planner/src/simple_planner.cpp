#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class SimplePlanner : public rclcpp::Node {
  
  private:
  
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map;
  
    void goal_received_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      printf("goal received\n");
      RCLCPP_INFO(this->get_logger(), "I heard the pose: '%f %f %f'", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    void map_received_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      printf("map received\n");
      RCLCPP_INFO(this->get_logger(), "I heard the map: '%d %d %f'", msg->info.width, msg->info.height, msg->info.resolution);
    }

  public:

    SimplePlanner() : Node("simple_planner") {
      sub_goal =  this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(&SimplePlanner::goal_received_callback, this, std::placeholders::_1));

      map = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&SimplePlanner::map_received_callback, this, std::placeholders::_1));
    }
    
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePlanner>());
  rclcpp::shutdown();
  return 0;
}
