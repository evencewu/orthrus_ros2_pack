#include "orthrus_controller/algorithm/LeggedMpc.hpp"

namespace orthrus_controller
{
    void LeggedMpc::Init(std::shared_ptr<JointState> joint_ptr,
                         std::shared_ptr<OdomState> odom_ptr,
                         std::shared_ptr<std::vector<TouchState>> touch_ptr,
                         std::shared_ptr<PinocchioInterface> pinocchio_ptr)
    {
        joint_state_ = joint_ptr;
        odom_state_ = odom_ptr;
        touch_state_ = touch_ptr;

        pinocchio_interface_ = pinocchio_ptr;
    }

    void LeggedMpc::Update(rclcpp::Time time, rclcpp::Duration duration)
    {
    }

}