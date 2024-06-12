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

        body2footforce_mat_ = Eigen::MatrixXd::Random(6, 12);
        body2footforce_mat_plus_ = Eigen::MatrixXd::Random(6, 12);
    }

    void LeggedMpc::Update(rclcpp::Time time, rclcpp::Duration duration)
    {
        Eigen::VectorXd body_force(6);

        // 直接赋值初始化
        body_force << 0, 0, 5, 0, 0, 0;

        Eigen::VectorXd foot_force = Body2FootForce(body_force);

        //(*touch_state_)[0].touch_force = foot_force.segment<3>(0);
        //(*touch_state_)[1].touch_force = foot_force.segment<3>(3);
        //(*touch_state_)[2].touch_force = foot_force.segment<3>(6);
        //(*touch_state_)[3].touch_force = foot_force.segment<3>(9);
        //
        // torq_ = Foot2JointForce();
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

            torq.segment<3>(foot_num * 3) = torq_v12.segment<3>(foot_num * 3);
        }

        return torq + gravity_torq;
    }

    Eigen::VectorXd LeggedMpc::Body2FootForce(Eigen::VectorXd body_force)
    {
        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            body2footforce_mat_.block<3, 3>(0, foot_num * 3) = Eigen::Matrix3d::Identity();
            body2footforce_mat_.block<3, 3>(3, foot_num * 3) = VectorToSkewSymmetricMatrix((*touch_state_)[foot_num].touch_position);
        }

        // 进行 SVD 分解
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(body2footforce_mat_, Eigen::ComputeThinU | Eigen::ComputeThinV);

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

        // 计算 Mat 的伪逆 Mat+
        body2footforce_mat_plus_ = V * Sigma_inv * U.transpose();

        return body2footforce_mat_plus_ * body_force;

        //body2footforce_mat_ = Eigen::MatrixXd::Random(6, 12);
//
        //for (int foot_num = 0; foot_num < 4; foot_num++)
        //{
        //    body2footforce_mat_.block<3, 3>(0, foot_num * 3) = (*touch_state_)[foot_num].touch_position.asDiagonal();
        //    body2footforce_mat_.block<3, 3>(3, foot_num * 3) = VectorToSkewSymmetricMatrix((*touch_state_)[foot_num].touch_position);
        //}

        // body2footforce_mat_plus_ = GetMinimumTworamTMat(body2footforce_mat_);

        // return body2footforce_mat_plus_ * body_force;
    }

    Eigen::MatrixXd GetMinimumTworamTMat(Eigen::MatrixXd input)
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

    std::stringstream LeggedMpc::Logger()
    {
        std::stringstream ss;
        ss << body2footforce_mat_ << std::endl;
        ss << body2footforce_mat_plus_ << std::endl;
        return ss;
    }

}