#include "orthrus_controllers/orthrus_cmd.hpp"

namespace orthrus_cmd
{
    OrthrusCmdNode::OrthrusCmdNode() : Node("orthrus_cmd")
    {
        goal_pose_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&OrthrusCmdNode::MainLoop, this));
    }

    void OrthrusCmdNode::MainLoop()
    {
        goal_pose_stamped_msg_.header.stamp = this->now();
        goal_pose_stamped_msg_.header.frame_id = "odom";

        goal_pose_stamped_msg_.pose.position.x = 1;
        goal_pose_stamped_msg_.pose.position.y = 1;
        goal_pose_stamped_msg_.pose.position.z = 1;

        goal_pose_stamped_msg_.pose.orientation.w = 1;
        goal_pose_stamped_msg_.pose.orientation.x = 0;
        goal_pose_stamped_msg_.pose.orientation.y = 0;
        goal_pose_stamped_msg_.pose.orientation.z = 0;

        goal_pose_stamped_pub_->publish(goal_pose_stamped_msg_);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_cmd::OrthrusCmdNode>());
    rclcpp::shutdown();
    return 0;
}