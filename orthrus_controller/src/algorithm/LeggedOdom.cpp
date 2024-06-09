#include "orthrus_controller/algorithm/LeggedOdom.hpp"

namespace orthrus_controller
{
    void LeggedOdom::Init(std::shared_ptr<OdomState> odom_ptr,
                          std::shared_ptr<std::vector<TouchState>> touch_ptr)
    {
        touch_state_ = touch_ptr;
        odom_state_ = odom_ptr;
    }

    void LeggedOdom::Update(rclcpp::Time time)
    {
        odom_state_->euler = Quaternion2Euler(odom_state_->imu.orientation);
    }

    Eigen::Vector3d LeggedOdom::Quaternion2Euler(Eigen::Quaterniond quat)
    {
        Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX
        return euler_angles;
    }

}
