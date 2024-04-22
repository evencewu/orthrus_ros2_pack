#include "orthrus_controllers/interface/OrthrusInterface.hpp"

namespace orthrus_control
{
    OrthrusInterface::OrthrusInterface(const rclcpp::Node::SharedPtr &node)
    {
        node_ = node;
    }

    OrthrusInterface::~OrthrusInterface()
    {
    }

    void OrthrusInterface::Init()
    {
        RCLCPP_INFO(node_->get_logger(), "OrthrusInterface init");
        orthrus_joint_state_sub_ = node_->create_subscription<orthrus_interfaces::msg::OrthrusJointState>("/orthrus_interface/joint_state", 10, std::bind(&OrthrusInterface::OrthrusJointStateSubCallback, this, std::placeholders::_1));
        orthrus_viewer_joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/orthrus_viewer/joint_state", 10);

        orthrus_imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("/orthrus_interface/imu", 10, std::bind(&OrthrusInterface::OrthrusImuSubCallback, this, std::placeholders::_1));
        orthrus_viewer_horizontal_pub_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    }

    void OrthrusInterface::OrthrusJointStateSubCallback(const orthrus_interfaces::msg::OrthrusJointState::SharedPtr msg)
    {
        orthrus_viewer_joint_state_msg_.header.stamp = node_->now();
        orthrus_viewer_joint_state_msg_.header.frame_id = "body";

        std::vector<double> position = std::vector<double>(12);
        std::vector<double> velocity = std::vector<double>(12);
        std::vector<double> effort = std::vector<double>(12);

        orthrus_viewer_joint_state_msg_.name = joint_name_;

        for (int i = 0; i < 12; i++)
        {
            // OrthrusParam_.joint[i].position = msg->motor[i].pos;
            // OrthrusParam_.joint[i].velocity = msg->motor[i].vec;
            // OrthrusParam_.joint[i].effort = msg->motor[i].torq;

            position[i] = msg->motor[i].pos;
            velocity[i] = msg->motor[i].vec;
            effort[i] = msg->motor[i].torq;
        }

        orthrus_viewer_joint_state_msg_.position = position;
        orthrus_viewer_joint_state_msg_.velocity = velocity;
        orthrus_viewer_joint_state_msg_.effort = effort;

        orthrus_viewer_joint_state_pub_->publish(orthrus_viewer_joint_state_msg_);
    }

    void OrthrusInterface::OrthrusImuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        
        geometry_msgs::msg::TransformStamped tf_stamped;
        // orthrus_viewer_horizontal_pub_
        tf_stamped.header.stamp = node_->now();

        tf_stamped.header.frame_id = "horizontal";
        tf_stamped.child_frame_id = "body";

        tf_stamped.transform.translation.x = 0.0;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = 0.0;

        tf_stamped.transform.rotation.x = msg->orientation.x;
        tf_stamped.transform.rotation.y = msg->orientation.y;
        tf_stamped.transform.rotation.z = msg->orientation.z;
        tf_stamped.transform.rotation.w = msg->orientation.w;

        orthrus_viewer_horizontal_msg_.transforms.clear();

        orthrus_viewer_horizontal_msg_.transforms.push_back(tf_stamped);

        orthrus_viewer_horizontal_pub_->publish(orthrus_viewer_horizontal_msg_);
    }
}