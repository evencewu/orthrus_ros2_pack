#include "orthrus_control/calibration/CalibrationVisualization.hpp"

namespace orthrus_control
{
    void CalibrationVisualization::Init(std::shared_ptr<OrthrusControlVariable> assembly_ptr)
    {
        assembly_ = assembly_ptr;
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

            tf_stamped.transform.rotation.w = assembly_->leg[i].imu.unified_gyro_.w();
            tf_stamped.transform.rotation.x = assembly_->leg[i].imu.unified_gyro_.x();
            tf_stamped.transform.rotation.y = assembly_->leg[i].imu.unified_gyro_.y();
            tf_stamped.transform.rotation.z = assembly_->leg[i].imu.unified_gyro_.z();

            leg_imu_msg_.transforms.push_back(tf_stamped);

            leg_imu_publisher_->publish(leg_imu_msg_);            
        }
    }
}