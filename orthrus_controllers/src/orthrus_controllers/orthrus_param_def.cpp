#include "orthrus_controllers/orthrus_param_def.hpp"

namespace othrus_ctrl
{
    othrus_parma_def::othrus_parma_def()
    {
        // You should change here to set up your own URDF file or just pass it as an argument of this example.
        const std::string urdf_filename = std::string("/home/evence/code_file/ros2_ws/orthrus/src/orthrus_ros2_pack/orthrus_sim/orthrus_gazebo/models/orthrus/urdf/orthrus.urdf.xacro");

        // Load the urdf model
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_filename, model);
    }
}
