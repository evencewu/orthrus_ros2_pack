#include "orthrus_controller/algorithm/LeggedOdom.hpp"

namespace orthrus_controller
{
    void LeggedOdom::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
    }

    void LeggedOdom::calibration(rclcpp::Time time, rclcpp::Duration duration)
    {
    }

    void LeggedOdom::Update(rclcpp::Time time, rclcpp::Duration duration)
    {
        orthrus_interfaces_->odom_state.imu = orthrus_interfaces_->robot_state.body_imu;
        orthrus_interfaces_->odom_state.euler = Quaternion2Euler(orthrus_interfaces_->odom_state.imu.orientation);

        double dt = duration.seconds();

        Eigen::Vector3d world_acc = orthrus_interfaces_->odom_state.imu.orientation * orthrus_interfaces_->odom_state.imu.linear_acceleration;
        
        // 积分得到速度
        orthrus_interfaces_->odom_state.velocity += world_acc * dt;

        // 积分得到位置
        orthrus_interfaces_->odom_state.position += orthrus_interfaces_->odom_state.velocity * dt;
    }

    Eigen::Vector3d LeggedOdom::Quaternion2Euler(Eigen::Quaterniond quat)
    {
        Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX
        return euler_angles;
    }
}
