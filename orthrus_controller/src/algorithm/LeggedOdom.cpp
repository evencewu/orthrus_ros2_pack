#include "orthrus_controller/algorithm/LeggedOdom.hpp"

namespace orthrus_controller
{
    void LeggedOdom::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
    }

    void LeggedOdom::Calibration(rclcpp::Time time, rclcpp::Duration duration)
    {
    }

    void LeggedOdom::Update(rclcpp::Time time, rclcpp::Duration duration)
    {
        orthrus_interfaces_->odom_state.imu = orthrus_interfaces_->robot_state.body_imu;
        orthrus_interfaces_->odom_state.euler = Quaternion2Euler(orthrus_interfaces_->odom_state.imu.orientation);

        orthrus_interfaces_->odom_state.dt = (double)(duration.nanoseconds()) / 1000000000;

        double dt = orthrus_interfaces_->odom_state.dt;

        VelocityPredict(time,duration);

        // 积分得到速度
        

        // 积分得到位置
        orthrus_interfaces_->odom_state.imu_position += orthrus_interfaces_->odom_state.imu_velocity * dt;
    }

    void LeggedOdom::VelocityPredict(rclcpp::Time time, rclcpp::Duration duration)
    {
        double dt = orthrus_interfaces_->odom_state.dt;

        Eigen::Matrix3d rotation_matrix = orthrus_interfaces_->odom_state.imu.orientation.toRotationMatrix();
        // imu速度分量预测
        orthrus_interfaces_->odom_state.imu_acceleration = rotation_matrix * orthrus_interfaces_->odom_state.imu.linear_acceleration + orthrus_interfaces_->odom_state.gravity;

        orthrus_interfaces_->odom_state.imu_velocity += orthrus_interfaces_->odom_state.imu_acceleration * dt;
    }

    Eigen::Vector3d LeggedOdom::Quaternion2Euler(Eigen::Quaterniond quat)
    {
        Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX
        return euler_angles;
    }
}
