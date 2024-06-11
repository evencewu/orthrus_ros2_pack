#include "orthrus_controller/interfaces/PinocchioInterface.hpp"

namespace orthrus_controller
{
    void PinocchioInterface::Init(std::shared_ptr<JointState> joint_ptr,
                                  std::shared_ptr<std::vector<TouchState>> touch_ptr)
    {

        joint_state_ = joint_ptr;
        touch_state_ = touch_ptr;

        pinocchio::urdf::buildModel(urdf_filename_, model_);

        pinocchio::urdf::buildGeom(model_, urdf_filename_, pinocchio::COLLISION, collision_model_, mesh_dir_);
        pinocchio::urdf::buildGeom(model_, urdf_filename_, pinocchio::VISUAL, visual_model_, mesh_dir_);

        collision_data_ = pinocchio::GeometryData(collision_model_);
        visual_data_ = pinocchio::GeometryData(visual_model_);

        // Sample a random configuration
        data_ = pinocchio::Data(model_);
        joint_ = pinocchio::neutral(model_);

        // Perform the forward kinematics over the kinematic tree
        pinocchio::forwardKinematics(model_, data_, joint_);
        pinocchio::framesForwardKinematics(model_, data_, joint_);
        pinocchio::updateGlobalPlacements(model_, data_);

        // Update Geometry models
        pinocchio::updateGeometryPlacements(model_, data_, collision_model_, collision_data_);
        pinocchio::updateGeometryPlacements(model_, data_, visual_model_, visual_data_);
    }

    void PinocchioInterface::Update(rclcpp::Time time)
    {
        joint_ = Eigen::VectorXd::Map(joint_state_->position.data(), joint_state_->position.size());
        // 执行正向运动学
        pinocchio::forwardKinematics(model_, data_, joint_);
        pinocchio::framesForwardKinematics(model_, data_, joint_);

        // 更新模型中所有关节的位姿
        pinocchio::updateGlobalPlacements(model_, data_);

        pinocchio::updateGeometryPlacements(model_, data_, collision_model_, collision_data_);
        pinocchio::updateGeometryPlacements(model_, data_, visual_model_, visual_data_);

        FootPositionCalculation();

        /*
        // 设置重力向量，假设沿着 -z 方向
        model.gravity.linear(Eigen::Vector3d(0, 0, -9.81));

        // 使用 RNEA 算法计算重力补偿力矩
        Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);

        // 打印重力补偿力矩
        std::cout << "Gravity compensation torques: \n"
                  << tau << std::endl;
                  */
    }

    std::stringstream PinocchioInterface::Logger()
    {
        std::stringstream ss;

        pinocchio::FrameIndex frame_index = model_.getFrameId("LF_FOOT");

        // Get the position of the frame in the world coordinate system
        const pinocchio::SE3 &frame_position = data_.oMf[frame_index];

        ss << "Translation: " << frame_position.translation().transpose() << std::endl;
        // for (size_t i = 0; i < model_.frames.size(); ++i)
        //{
        //     const pinocchio::Frame &frame = model_.frames[i];
        //     ss << "Frame " << i << ": " << std::endl;
        //     ss << "  Name: " << frame.name << std::endl;
        //     ss << "  Parent joint ID: " << frame.parent << std::endl;
        //     ss << "  Frame type: " << frame.type << std::endl;
        // }

        // for (int joint_id = 1; joint_id < 12; joint_id++)
        //{
        //     ss << model_.names[joint_id] << " " << joint_[joint_id] << std::endl;
        // }

        return ss;
    }

    void PinocchioInterface::FootPositionCalculation()
    {
        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            pinocchio::FrameIndex frame_index = model_.getFrameId(foot_name_[foot_num]);
            // Get the position of the frame in the world coordinate system
            const pinocchio::SE3 &frame_position = data_.oMf[frame_index];
            (*touch_state_)[foot_num].touch_position = frame_position.translation();
        }
    }

    Eigen::MatrixXd PinocchioInterface::GetJacobianMatrix(std::string frame_name)
    {
        pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
        //  计算雅可比矩阵
        Eigen::MatrixXd J(6, model_.nv); // model_.nv nq

        J = computeJointJacobians(model_, data_, joint_);

        return J;
    }

    Eigen::VectorXd PinocchioInterface::LegGravityCompensation()
    {
        // 定义机器人的配置（关节角度），速度和加速度
        Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv); // 速度为零
        // Eigen::VectorXd tau = Eigen::VectorXd::Zero(model_.nv); // 定义关节力矩
        Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv); // 关节加速度
        // tau[0] = 1.0;                                          // 假设第一个关节力矩为1

        Eigen::VectorXd tau = pinocchio::rnea(model_, data_, joint_, v, a);

        return tau;
    }
}
