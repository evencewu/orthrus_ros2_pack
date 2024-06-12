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

        MarkVisualization(time);
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

        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            tf_stamped.child_frame_id = foot_names_[foot_num];
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

    void OrthrusVisualization::MarkVisualization(rclcpp::Time time)
    {
        markerarray_msg_.markers.clear();

        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = foot_names_[foot_num];
            marker.header.stamp = time;
            marker.ns = foot_names_[foot_num];
            marker.id = foot_num;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.points.resize(2);
            marker.points[0].x = 0.0;
            marker.points[0].y = 0.0;
            marker.points[0].z = 0.0;
            marker.points[1].x = (*touch_state_)[foot_num].touch_force[0] / 100;
            marker.points[1].y = (*touch_state_)[foot_num].touch_force[1] / 100;
            marker.points[1].z = (*touch_state_)[foot_num].touch_force[2] / 100;

            // 设置箭头的缩放（箭头的大小）
            marker.scale.x = 0.01; // 箭头的长度
            marker.scale.y = 0.01; // 箭头的宽度
            marker.scale.z = 0.01; // 箭头的高度

            // 设置箭头的颜色
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            markerarray_msg_.markers.push_back(marker);
        }

        marker_publisher_->publish(markerarray_msg_);
    }
}
