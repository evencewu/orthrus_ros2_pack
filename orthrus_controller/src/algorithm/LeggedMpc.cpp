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
        (*touch_state_)[0].touch_force = Eigen::Vector3d(3,0,0);
        (*touch_state_)[1].touch_force = Eigen::Vector3d(3,0,0);
        (*touch_state_)[2].touch_force = Eigen::Vector3d(3,0,0);
        (*touch_state_)[3].touch_force = Eigen::Vector3d(3,0,0);

        torq_ =  Foot2JointForce();
    }

    Eigen::VectorXd LeggedMpc::Foot2JointForce()
    {
        Eigen::VectorXd gravity_torq = pinocchio_interface_->LegGravityCompensation();
        Eigen::VectorXd torq(12);

        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            Eigen::VectorXd torq_v12;
            Eigen::VectorXd pos(6);
            pos.setZero(); // 可选的初始化为零向量
            pos.segment<3>(0) = (*touch_state_)[foot_num].touch_force;

            Eigen::MatrixXd j = pinocchio_interface_->GetJacobianMatrix(foot_name_[foot_num]);
            torq_v12 = j.transpose() * pos;

            torq.segment<3>(foot_num*3) = torq_v12.segment<3>(foot_num*3);
        }

        return torq + gravity_torq;
    }

}