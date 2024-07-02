#include "orthrus_controller/visualization/OrthrusVisualization.hpp"

namespace orthrus_controller
{
    void OrthrusVisualization::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
    }

    void OrthrusVisualization::Update(rclcpp::Time time)
    {
        ModelVisualization(time);
        TfVisualization(time);
        MarkVisualization(time);
        ImuVisualization(time);
    }

    void OrthrusVisualization::TfVisualization(rclcpp::Time time)
    {
        tf_msg_.transforms.clear();
        BodyTfVisualization(time);
        FootTfVisualization(time);
        tf_publisher_->publish(tf_msg_);
    }

    void OrthrusVisualization::BodyTfVisualization(rclcpp::Time time)
    {
        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header.stamp = time;
        tf_stamped.header.frame_id = "odom";
        tf_stamped.child_frame_id = "base";

        tf_stamped.transform.translation.x = 0.0;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = 0.0;

        tf_stamped.transform.rotation.w = orthrus_interfaces_->odom_state.imu.orientation.w();
        tf_stamped.transform.rotation.x = orthrus_interfaces_->odom_state.imu.orientation.x();
        tf_stamped.transform.rotation.y = orthrus_interfaces_->odom_state.imu.orientation.y();
        tf_stamped.transform.rotation.z = orthrus_interfaces_->odom_state.imu.orientation.z();

        tf_msg_.transforms.push_back(tf_stamped);
    }

    void OrthrusVisualization::FootTfVisualization(rclcpp::Time time)
    {
        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header.stamp = time;
        tf_stamped.header.frame_id = "base";

        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            tf_stamped.child_frame_id = foot_names_[foot_num];
            tf_stamped.transform.translation.x = orthrus_interfaces_->odom_state.touch_state[foot_num].touch_position[0];
            tf_stamped.transform.translation.y = orthrus_interfaces_->odom_state.touch_state[foot_num].touch_position[1];
            tf_stamped.transform.translation.z = orthrus_interfaces_->odom_state.touch_state[foot_num].touch_position[2];
            tf_stamped.transform.rotation.w = 0;
            tf_stamped.transform.rotation.x = 0;
            tf_stamped.transform.rotation.y = 0;
            tf_stamped.transform.rotation.z = 0;

            tf_msg_.transforms.push_back(tf_stamped);
        }

        // marker.points[1].x = orthrus_interfaces_->odom_state.imu_acceleration[0] / 10;
        // marker.points[1].y = orthrus_interfaces_->odom_state.imu_acceleration[1] / 10;
        // marker.points[1].z = orthrus_interfaces_->odom_state.imu_acceleration[2] / 10;
    }

    void OrthrusVisualization::ModelVisualization(rclcpp::Time time)
    {
        joint_state_msg_.header.stamp = time;
        joint_state_msg_.header.frame_id = "body";

        // Direct initialization with zeros
        joint_state_msg_.position = orthrus_interfaces_->robot_state.joint.position;
        joint_state_msg_.velocity = orthrus_interfaces_->robot_state.joint.velocity;
        joint_state_msg_.effort = orthrus_interfaces_->robot_state.joint.effort;

        joint_state_msg_.name = joint_name_;
        joint_state_publisher_->publish(joint_state_msg_);
    }

    void OrthrusVisualization::MarkVisualization(rclcpp::Time time)
    {
        markerarray_msg_.markers.clear();
        //

        LegForceMarkVisualization(time);

        // 加速度方向可视化
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "base";
        marker.header.stamp = time;
        marker.ns = "velocity";
        marker.id = 4;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.points.resize(2);
        marker.points[0].x = 0.0;
        marker.points[0].y = 0.0;
        marker.points[0].z = 0.0;
        marker.points[1].x = orthrus_interfaces_->odom_state.imu_velocity[0] / 10;
        marker.points[1].y = orthrus_interfaces_->odom_state.imu_velocity[1] / 10;
        marker.points[1].z = orthrus_interfaces_->odom_state.imu_velocity[2] / 10;

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

        //

        marker_publisher_->publish(markerarray_msg_);
    }

    void OrthrusVisualization::LegForceMarkVisualization(rclcpp::Time time)
    {
        visualization_msgs::msg::Marker marker;

        // 足尖力分配可视化
        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            marker = visualization_msgs::msg::Marker();

            marker.header.frame_id = foot_names_[foot_num] + "_link";
            marker.header.stamp = time;
            marker.ns = foot_names_[foot_num];
            marker.id = foot_num;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.points.resize(2);
            marker.points[0].x = 0.0;
            marker.points[0].y = 0.0;
            marker.points[0].z = 0.0;
            marker.points[1].x = orthrus_interfaces_->odom_state.touch_state[foot_num].touch_force[0] / 10;
            marker.points[1].y = orthrus_interfaces_->odom_state.touch_state[foot_num].touch_force[1] / 10;
            marker.points[1].z = orthrus_interfaces_->odom_state.touch_state[foot_num].touch_force[2] / 10;

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
    }

    void OrthrusVisualization::ImuVisualization(rclcpp::Time time)
    {
        imu_msg_.header.stamp = time;
        imu_msg_.header.frame_id = "odom";

        // 填充IMU数据
        imu_msg_.orientation.x = orthrus_interfaces_->odom_state.imu.orientation.x();
        imu_msg_.orientation.y = orthrus_interfaces_->odom_state.imu.orientation.y();
        imu_msg_.orientation.z = orthrus_interfaces_->odom_state.imu.orientation.z();
        imu_msg_.orientation.w = orthrus_interfaces_->odom_state.imu.orientation.w();

        imu_msg_.angular_velocity.x = orthrus_interfaces_->odom_state.imu.angular_velocity[0];
        imu_msg_.angular_velocity.y = orthrus_interfaces_->odom_state.imu.angular_velocity[1];
        imu_msg_.angular_velocity.z = orthrus_interfaces_->odom_state.imu.angular_velocity[2];

        imu_msg_.linear_acceleration.x = orthrus_interfaces_->odom_state.imu.linear_acceleration[0];
        imu_msg_.linear_acceleration.y = orthrus_interfaces_->odom_state.imu.linear_acceleration[1];
        imu_msg_.linear_acceleration.z = orthrus_interfaces_->odom_state.imu.linear_acceleration[2];

        imu_publisher_->publish(imu_msg_);
    }
}
