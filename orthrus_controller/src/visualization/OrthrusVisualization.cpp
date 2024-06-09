#include "orthrus_controller/visualization/OrthrusVisualization.hpp"

namespace orthrus_controller
{
    void OrthrusVisualization::Init(std::shared_ptr<JointState> joint_ptr,
                                    std::shared_ptr<OdomState> odom_ptr,
                                    std::shared_ptr<std::vector<TouchState>> touch_ptr)
    {
        // JointState joint_state_;
        touch_state_ = touch_ptr;
        joint_state_ = joint_ptr;
        odom_state_ = odom_ptr;
    }

    void OrthrusVisualization::Update(rclcpp::Time time)
    {
        odom_msg_.transforms.clear();
        ModelVisualization(time);
        ImuVisualization(time);
        FootPointVisualization(time);
        odom_publisher_->publish(odom_msg_);
    }

    void OrthrusVisualization::ModelVisualization(rclcpp::Time time)
    {
        joint_state_msg_.header.stamp = time;
        joint_state_msg_.header.frame_id = "body";

        // Direct initialization with zeros
        joint_state_msg_.position = joint_state_->position;
        joint_state_msg_.velocity = joint_state_->velocity;
        joint_state_msg_.effort = joint_state_->effort;

        joint_state_msg_.name = joint_name_;
        joint_state_publisher_->publish(joint_state_msg_);
    }
    void OrthrusVisualization::ImuVisualization(rclcpp::Time time)
    {
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

        
        odom_msg_.transforms.push_back(tf_stamped);
    }

    void OrthrusVisualization::FootPointVisualization(rclcpp::Time time)
    {
        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header.stamp = time;
        tf_stamped.header.frame_id = "base";

        std::vector<std::string> foot_names = {"LF_foot", "LH_foot", "RF_foot", "RH_foot"};

        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            tf_stamped.child_frame_id = foot_names[foot_num];
            tf_stamped.transform.translation.x = (*touch_state_)[foot_num].touch_position[0];
            tf_stamped.transform.translation.y = (*touch_state_)[foot_num].touch_position[1];
            tf_stamped.transform.translation.z = (*touch_state_)[foot_num].touch_position[2];
            tf_stamped.transform.rotation.w = -odom_state_->imu.orientation.w();
            tf_stamped.transform.rotation.x = odom_state_->imu.orientation.x();
            tf_stamped.transform.rotation.y = odom_state_->imu.orientation.y();
            tf_stamped.transform.rotation.z = odom_state_->imu.orientation.z();
            odom_msg_.transforms.push_back(tf_stamped);
        }
    }
}
