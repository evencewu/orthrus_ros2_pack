#include "orthrus_controllers/orthrus_param_def.hpp"

namespace othrus_ctrl
{
    othrus_parma_def::othrus_parma_def();
    {
        // You should change here to set up your own URDF file or just pass it as an argument of this example.
        const std::string urdf_filename = (argc <= 1) ? PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf") : argv[1];

        // Load the urdf model
        pinocchio::Model model;
        pinocchio::pinocchio::urdf::buildModel(urdf_filename, model);
    }
}
