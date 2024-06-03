#include "orthrus_controller/interfaces/PinocchioInterface.hpp"

namespace orthrus_controller
{
    void PinocchioInterface::Init()
    {
        
        //pinocchio::urdf::buildGeom(model_, urdf_filename_, COLLISION, collision_model_, mesh_dir_);

        
        //pinocchio::urdf::buildGeom(model_, urdf_filename_, VISUAL, visual_model_, mesh_dir_);
        //std::cout << "model name: " << model.name << std::endl;

        // Create data required by the algorithms
        //data_ = pinocchio::Data(model_);
        //collision_data_ = pinocchio::GeometryData(collision_model_);
        //visual_data_ = pinocchio::GeometryData(visual_model_);

        // Sample a random configuration
        //q_ = pinocchio::randomConfiguration(model_);

        // Perform the forward kinematics over the kinematic tree
        //pinocchio::forwardKinematics(model_, data_, q_);

        // Update Geometry models
        //pinocchio::updateGeometryPlacements(model_, data_, collision_model_, collision_data_);
        //pinocchio::updateGeometryPlacements(model_, data_, visual_model_, visual_data_);

        // Print out the placement of each joint of the kinematic tree
        //std::cout << "\nJoint placements:" << std::endl;
        //for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
        //    std::cout << std::setw(24) << std::left << model.names[joint_id] << ": " << std::fixed
        //              << std::setprecision(2) << data.oMi[joint_id].translation().transpose() << std::endl;

        // Print out the placement of each collision geometry object
        //std::cout << "\nCollision object placements:" << std::endl;
        //for (GeomIndex geom_id = 0; geom_id < (GeomIndex)collision_model.ngeoms; ++geom_id)
        //    std::cout << geom_id << ": " << std::fixed << std::setprecision(2)
        //              << collision_data.oMg[geom_id].translation().transpose() << std::endl;

        // Print out the placement of each visual geometry object
        //std::cout << "\nVisual object placements:" << std::endl;
        //for (GeomIndex geom_id = 0; geom_id < (GeomIndex)visual_model.ngeoms; ++geom_id)
        //    std::cout << geom_id << ": " << std::fixed << std::setprecision(2)
        //            << visual_data.oMg[geom_id].translation().transpose() << std::endl;
    }

}
