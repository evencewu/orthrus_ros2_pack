#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace orthrus_cmd
{
    class OrthrusCmdNode : public rclcpp::Node
    {
    public:
        OrthrusCmdNode();

        rclcpp::TimerBase::SharedPtr timer_;
    private:
        void MainLoop();

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_stamped_pub_;
        geometry_msgs::msg::PoseStamped goal_pose_stamped_msg_;
    };
}