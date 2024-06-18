#include "orthrus_controller/algorithm/LeggedMpc.hpp"

namespace orthrus_controller
{
    void LeggedMpc::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr,
                         std::shared_ptr<PinocchioInterfaces> pinocchio_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
        pinocchio_interfaces_ = pinocchio_ptr;
    }

    void LeggedMpc::Update(rclcpp::Time time, rclcpp::Duration duration)
    {
        // 更新pinocchio动力学参数
        

        // 直接赋值初始化
        GetBodyForcePD();
        orthrus_interfaces_->robot_target.target_body_force << 0, 0, 200, 0, 0, 0;

        std::vector<bool> gait = orthrus_interfaces_->robot_target.gait_sequence[orthrus_interfaces_->robot_target.gait_num];

        Eigen::VectorXd foot_force = Body2FootForce(orthrus_interfaces_->robot_target.target_body_force, gait);

        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            pinocchio::FrameIndex frame_index = pinocchio_interfaces_->model_.getFrameId(foot_name_[foot_num]);
            orthrus_interfaces_->odom_state.touch_state[foot_num].touch_force = orthrus_interfaces_->odom_state.touch_state[foot_num].touch_rotation.transpose() * foot_force.segment<3>(foot_num*3);
        }

        orthrus_interfaces_->robot_cmd.effort = Foot2JointForce();
    }

    // 计算反作用力所需的关节力矩
    Eigen::VectorXd LeggedMpc::Foot2JointForce()
    {
        Eigen::VectorXd gravity_torq = pinocchio_interfaces_->LegGravityCompensation();
        Eigen::VectorXd torq(12);

        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            Eigen::VectorXd torq_v12;
            Eigen::VectorXd pos(6);
            pos.setZero(); // 可选的初始化为零向量
            pos.segment<3>(0) = orthrus_interfaces_->odom_state.touch_state[foot_num].touch_force;

            Eigen::MatrixXd j = pinocchio_interfaces_->GetJacobianMatrix(foot_name_[foot_num]);
            torq_v12 = j.transpose() * pos;

            torq.segment<3>(foot_num * 3) = torq_v12.segment<3>(foot_num * 3);
        }

        // return torq + gravity_torq;
        return torq;
    }

    Eigen::VectorXd LeggedMpc::Body2FootForce(Eigen::VectorXd body_force, std::vector<bool> gait_touch_sequence)
    {
        body2footforce_mat_ = Eigen::MatrixXd::Zero(6, 12);
        body2footforce_mat_plus_ = Eigen::MatrixXd::Zero(6, 12);
        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            if (gait_touch_sequence[foot_num])
            {
                body2footforce_mat_.block<3, 3>(0, foot_num * 3) = Eigen::Matrix3d::Identity();
                body2footforce_mat_.block<3, 3>(3, foot_num * 3) = VectorToSkewSymmetricMatrix(orthrus_interfaces_->odom_state.touch_state[foot_num].touch_position);
            }
        }

        body2footforce_mat_plus_ = GetMinimumTworamTMat(body2footforce_mat_);

        return body2footforce_mat_plus_ * body_force;
    }

    Eigen::MatrixXd LeggedMpc::GetMinimumTworamTMat(Eigen::MatrixXd input)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(input, Eigen::ComputeThinU | Eigen::ComputeThinV);

        // 获取 U, S, V 矩阵
        Eigen::MatrixXd U = svd.matrixU();
        Eigen::MatrixXd V = svd.matrixV();
        Eigen::VectorXd S = svd.singularValues();

        //// 构造 S 的伪逆
        Eigen::VectorXd S_inv(S.size());
        for (int i = 0; i < S.size(); ++i)
        {
            if (S(i) > 1e-10)
            { // 处理奇异值接近于0的情况
                S_inv(i) = 1.0 / S(i);
            }
            else
            {
                S_inv(i) = 0.0;
            }
        }
        Eigen::MatrixXd Sigma_inv = S_inv.asDiagonal();

        Eigen::MatrixXd output = V * Sigma_inv * U.transpose();
        // 计算 Mat 的伪逆 Mat+
        return output;
    }

    Eigen::MatrixXd LeggedMpc::GetTMat(Eigen::MatrixXd input)
    {
        return input.inverse();
    }

    void LeggedMpc::GetBodyForcePD()
    {

        double kp_position = 1;
        double kd_position = 1;
        double kp_angular = 1;
        double kd_angular = 1;

        // error P
        Eigen::Vector3d error_position = orthrus_interfaces_->robot_target.target_position - orthrus_interfaces_->odom_state.position;
        Eigen::Vector3d error_angular = orthrus_interfaces_->robot_target.target_euler - orthrus_interfaces_->odom_state.euler;

        // error D
        Eigen::Vector3d error_velocity = orthrus_interfaces_->robot_target.target_velocity - orthrus_interfaces_->odom_state.velocity;
        Eigen::Vector3d error_angular_velocity = orthrus_interfaces_->robot_target.target_angular_velocity - orthrus_interfaces_->odom_state.angular_velocity;

        Eigen::Vector3d position = error_position * kp_position - error_velocity * kd_position;
        Eigen::Vector3d angular = error_angular * kp_angular - error_angular_velocity * kd_angular;

        orthrus_interfaces_->robot_target.target_body_force.segment<3>(0) = position;
        orthrus_interfaces_->robot_target.target_body_force.segment<3>(3) = angular;
    }

    Eigen::Matrix3d LeggedMpc::VectorToSkewSymmetricMatrix(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d V;
        V << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
        return V;
    }

    Eigen::Matrix3d LeggedMpc::VectorToDiagonalMatrix(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d V;
        V << v(0), 0, 0,
            0, v(1), 0,
            0, 0, v(2);
        return V;
    }

    std::stringstream LeggedMpc::Logger(int log_num)
    {
        std::stringstream ss;

        if (log_num = BODY2FOOTFORCE_LOG)
        {
            ss << body2footforce_mat_ << std::endl;
            ss << body2footforce_mat_plus_ << std::endl;
        }
        else if (JOINT_EFFOT_LOG)
        {
            ss << orthrus_interfaces_->robot_cmd.effort << std::endl;
        }

        return ss;
    }
}