#include "orthrus_controller/visualization/OrthrusVisualization.hpp"

namespace orthrus_controller
{
    void OrthrusVisualization::Init(std::shared_ptr<JointState> joint_ptr ,std::shared_ptr<OdomState> odom_ptr)
    {
        //JointState joint_state_;
        joint_state_ = joint_ptr;
        odom_state_ = odom_ptr;
    }

    void OrthrusVisualization::update(rclcpp::Time time)
    {
        joint_state_msg_.header.stamp = time;
        joint_state_msg_.header.frame_id = "body";

        // Direct initialization with zeros
        joint_state_msg_.position = joint_state_->position;
        joint_state_msg_.velocity = joint_state_->velocity;
        joint_state_msg_.effort = joint_state_->effort;

        joint_state_msg_.name = joint_name_;
        joint_state_publisher_->publish(joint_state_msg_);

        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header.stamp = time;
        tf_stamped.header.frame_id = "odom";
        tf_stamped.child_frame_id = "base";

        tf_stamped.transform.translation.x = 0.0;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = 0.0;
        tf_stamped.transform.rotation.w = odom_state_->imu.orientation.w();
        tf_stamped.transform.rotation.x = odom_state_->imu.orientation.x();
        tf_stamped.transform.rotation.y = odom_state_->imu.orientation.y();
        tf_stamped.transform.rotation.z = odom_state_->imu.orientation.z();

        odom_msg_.transforms.clear();
        odom_msg_.transforms.push_back(tf_stamped);
        odom_publisher_->publish(odom_msg_);
    }
    // Explicit instantiations for the expected node types
    // template OrthrusVisualization::OrthrusVisualization(const rclcpp::Node::SharedPtr& node, const std::vector<std::string>& joint_name);
    // template OrthrusVisualization::OrthrusVisualization(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::vector<std::string>& joint_name);
    // template void OrthrusVisualization::update(const rclcpp::Node::SharedPtr& node);
    // template void OrthrusVisualization::update(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
}
