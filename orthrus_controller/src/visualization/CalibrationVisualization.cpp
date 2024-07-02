#include "orthrus_controller/visualization/CalibrationVisualization.hpp"

namespace orthrus_controller
{
    void CalibrationVisualization::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
    }

    void CalibrationVisualization::Update(rclcpp::Time time)
    {
        leg_imu_msg_.transforms.clear();
        
        geometry_msgs::msg::TransformStamped tf_stamped;

        tf_stamped.header.stamp = time;

        for (int i = 0; i < 4; i++)
        {
            tf_stamped.header.frame_id = "odom";
            tf_stamped.child_frame_id = imu_names[i];

            tf_stamped.transform.translation.x = 0.0;
            tf_stamped.transform.translation.y = 0.0;
            tf_stamped.transform.translation.z = 0.0;

            tf_stamped.transform.rotation.w = orthrus_interfaces_->robot_state.leg_imu[i].orientation.w();
            tf_stamped.transform.rotation.x = orthrus_interfaces_->robot_state.leg_imu[i].orientation.x();
            tf_stamped.transform.rotation.y = orthrus_interfaces_->robot_state.leg_imu[i].orientation.y();
            tf_stamped.transform.rotation.z = orthrus_interfaces_->robot_state.leg_imu[i].orientation.z();

            leg_imu_msg_.transforms.push_back(tf_stamped);

            leg_imu_publisher_->publish(leg_imu_msg_);            
        }
    }
}