#include "orthrus_controller/visualization/OrthrusVisualization.hpp"

namespace orthrus_controller
{
    void OrthrusVisualization::Init(std::shared_ptr<JointParma> joint_parma_ptr)
    {
        //JointParma joint_parma_;
        joint_parma_ = joint_parma_ptr;
    }

    void OrthrusVisualization::update(rclcpp::Time time)
    {
        joint_state_msg_.header.stamp = time;
        joint_state_msg_.header.frame_id = "body";

        // Direct initialization with zeros
        joint_state_msg_.position = joint_parma_->position;
        joint_state_msg_.velocity = std::vector<double>(12, 0.0);
        joint_state_msg_.effort = std::vector<double>(12, 0.0);

        joint_state_msg_.name = joint_name_;
        joint_state_publisher_->publish(joint_state_msg_);

        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header.stamp = time;
        tf_stamped.header.frame_id = "odom";
        tf_stamped.child_frame_id = "base";

        tf_stamped.transform.translation.x = 0.0;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = 0.0;
        tf_stamped.transform.rotation.w = 1.0;
        tf_stamped.transform.rotation.x = 0.0;
        tf_stamped.transform.rotation.y = 0.0;
        tf_stamped.transform.rotation.z = 0.0;

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
