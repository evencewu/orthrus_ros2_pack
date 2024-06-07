#include "orthrus_controller/interfaces/PinocchioInterface.hpp"

namespace orthrus_controller
{
    void PinocchioInterface::Init(std::shared_ptr<JointParma> joint_parma_ptr)
    {

        joint_parma_ = joint_parma_ptr;

        pinocchio::urdf::buildModel(urdf_filename_, model_);

        pinocchio::urdf::buildGeom(model_, urdf_filename_, pinocchio::COLLISION, collision_model_, mesh_dir_);
        pinocchio::urdf::buildGeom(model_, urdf_filename_, pinocchio::VISUAL, visual_model_, mesh_dir_);

        // std::cout << "model name: " << model.name << std::endl;

        // Create data required by the algorithms
        data_ = pinocchio::Data(model_);
        collision_data_ = pinocchio::GeometryData(collision_model_);
        visual_data_ = pinocchio::GeometryData(visual_model_);

        // Sample a random configuration

        joint_ = pinocchio::neutral(model_);

        // Perform the forward kinematics over the kinematic tree
        pinocchio::forwardKinematics(model_, data_, joint_);

        // Update Geometry models
        pinocchio::updateGeometryPlacements(model_, data_, collision_model_, collision_data_);
        pinocchio::updateGeometryPlacements(model_, data_, visual_model_, visual_data_);
    }

    void PinocchioInterface::Update()
    {
        
        joint_ = Eigen::VectorXd::Map(joint_parma_->position.data(), joint_parma_->position.size());
        // 执行正向运动学
        pinocchio::forwardKinematics(model_, data_, joint_);

        // 更新模型中所有关节的位姿
        pinocchio::updateGlobalPlacements(model_, data_);
    }

    std::stringstream PinocchioInterface::Logger()
    {
        std::stringstream ss;
        for (int joint_id = 1; joint_id < 12; joint_id++)
        {
            ss << model_.names[joint_id] << " " << joint_[joint_id] << std::endl;
        }
        return ss;
    }

}
