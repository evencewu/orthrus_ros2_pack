#include "orthrus_controller/interfaces/PinocchioInterfaces.hpp"

namespace orthrus_controller
{
    void PinocchioInterfaces::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr,
                                   std::string path_name)
    {

        std::string mesh_dir_ = path_name + mesh_filename_;
        std::string urdf_path_ = path_name + urdf_filename_;

        orthrus_interfaces_ = orthrus_interfaces_ptr;

        // pinocchio::JointModelFreeFlyer root_joint;
        // pinocchio::urdf::buildModel(urdf_filename_, model_);

        pinocchio::urdf::buildModel(urdf_path_, pinocchio::JointModelFreeFlyer(), model_);

        collision_data_ = pinocchio::GeometryData(collision_model_);
        visual_data_ = pinocchio::GeometryData(visual_model_);

        // Sample a random configuration
        data_ = pinocchio::Data(model_);
        joint_position_ = pinocchio::neutral(model_);
        joint_velocity_ = Eigen::VectorXd::Zero(model_.nv);     // 速度为零
        joint_acceleration_ = Eigen::VectorXd::Zero(model_.nv); // 关节加速度

        // 设置重力方向

        // model_.jointPlacements[1] = pinocchio::SE3::Identity();

        // Perform the forward kinematics over the kinematic tree
        pinocchio::forwardKinematics(model_, data_, joint_position_);
        pinocchio::framesForwardKinematics(model_, data_, joint_position_);
        pinocchio::updateGlobalPlacements(model_, data_);

        // 广义重力项计算
        pinocchio::computeGeneralizedGravity(model_, data_, joint_position_);
        pinocchio::nonLinearEffects(model_, data_, joint_position_, joint_velocity_);
        pinocchio::computeCoriolisMatrix(model_, data_, joint_position_, joint_velocity_);

        // 雅可比矩阵计算
        pinocchio::computeJointJacobians(model_, data_, joint_position_);
    }

    void PinocchioInterfaces::Update(rclcpp::Time time)
    {
        // joint_[3] = orthrus_interfaces_->robot_state.body_imu.orientation.x();
        // joint_[4] = orthrus_interfaces_->robot_state.body_imu.orientation.y();
        // joint_[5] = orthrus_interfaces_->robot_state.body_imu.orientation.z();
        // joint_[6] = orthrus_interfaces_->robot_state.body_imu.orientation.w();

        joint_position_.segment<12>(7) = Eigen::VectorXd::Map(orthrus_interfaces_->robot_state.joint.position.data(), orthrus_interfaces_->robot_state.joint.position.size());

        // 执行正向运动学
        pinocchio::forwardKinematics(model_, data_, joint_position_);
        pinocchio::framesForwardKinematics(model_, data_, joint_position_);

        // 更新模型中所有关节的位姿
        pinocchio::updateGlobalPlacements(model_, data_);

        // 广义重力项计算
        pinocchio::computeGeneralizedGravity(model_, data_, joint_position_);
        pinocchio::nonLinearEffects(model_, data_, joint_position_, joint_velocity_);
        pinocchio::computeCoriolisMatrix(model_, data_, joint_position_, joint_velocity_);
        // 雅可比矩阵计算
        pinocchio::computeJointJacobians(model_, data_, joint_position_);

        FootPositionCalculation();

        // 设置重力向量，假设沿着 -z 方向

        // LegGravityCompensation();
    }

    void PinocchioInterfaces::FootPositionCalculation()
    {
        for (int foot_num = 0; foot_num < 4; foot_num++)
        {
            pinocchio::FrameIndex frame_index = model_.getFrameId(foot_name_[foot_num] + "_link");
            // Get the position of the frame in the world coordinate system
            const pinocchio::SE3 &frame_position = data_.oMf[frame_index];
            orthrus_interfaces_->odom_state.touch_state[foot_num].touch_position = frame_position.translation();

            pinocchio::FrameIndex base_index = model_.getFrameId("base");
            const pinocchio::SE3 &base_position = data_.oMf[frame_index];
            // base_position.rotation()

            // Eigen::Quaterniond quaternion(frame_position.rotation());
            orthrus_interfaces_->odom_state.touch_state[foot_num].touch_rotation = frame_position.rotation();
        }
    }

    Eigen::MatrixXd PinocchioInterfaces::GetJacobianMatrix(std::string frame_name)
    {
        pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
        //  计算雅可比矩阵
        Eigen::MatrixXd J(6, model_.nv); // model_.nv nq

        pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL, J);

        return J;
    }

    Eigen::VectorXd PinocchioInterfaces::GetJointEffortCompensation()
    {
        return this->data_.g.segment<12>(6);
    }

    Eigen::VectorXd PinocchioInterfaces::GetGravityCompensation()
    {
        return this->data_.g.segment<6>(0);
    }

    void PinocchioInterfaces::GetLagrange()
    {
    }

    std::stringstream PinocchioInterfaces::InitLogger()
    {
        std::stringstream ss;

        ss << "model_.nv: " << model_.nv << std::endl;
        for (size_t i = 0; i < model_.frames.size(); ++i)
        {
            const pinocchio::Frame &frame = model_.frames[i];
            ss << "Frame " << i << ": " << std::endl;
            ss << "  Name: " << frame.name << std::endl;
            ss << "  Parent joint ID: " << frame.parent << std::endl;
            ss << "  Frame type: " << frame.type << std::endl;
        }

        for (int joint_id = 1; joint_id < 14; joint_id++)
        {
            ss << joint_id << " " << model_.names[joint_id] << std::endl;
        }

        for (int joint_id = 0; joint_id < 19; joint_id++)
        {
            ss << joint_id << " " << joint_position_[joint_id] << std::endl;
        }

        ss << data_.g << std::endl;

        return ss;
    }

    std::stringstream PinocchioInterfaces::Logger()
    {
        std::stringstream ss;
        ss << data_.g << std::endl;
        return ss;
    }
}
