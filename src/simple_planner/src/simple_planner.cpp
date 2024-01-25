#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// geometry_msgs/msg/PoseStamped

class SimplePlanner : public rclcpp::Node {
  
  private:
  
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal;
  
    void goal_received_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      printf("goal received\n");
      RCLCPP_INFO(this->get_logger(), "I heard the pose: '%f %f %f'", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

  public:

    SimplePlanner() : Node("simple_planner") {
      sub_goal =  this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(&SimplePlanner::goal_received_callback, this, std::placeholders::_1));
    }
    
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePlanner>());
  rclcpp::shutdown();
  return 0;
}
