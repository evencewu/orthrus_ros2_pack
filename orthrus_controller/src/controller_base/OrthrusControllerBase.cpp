#include <orthrus_controller/controller_base/OrthrusControllerBase.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
namespace orthrus_controller
{
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  controller_interface::CallbackReturn OrthrusControllerBase::configure_joint(
      const std::vector<std::string> &joint_names,
      std::vector<JointHandle> &registered_handles)
  {
    auto logger = get_node()->get_logger();

    if (joint_names.empty())
    {
      RCLCPP_ERROR(logger, "No motor names specified");
      return controller_interface::CallbackReturn::ERROR;
    }

    // register handles
    registered_handles.reserve(joint_names.size());
    for (const auto &joint_name : joint_names)
    {
      const auto position_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&joint_name](const auto &interface)
          {
            return interface.get_prefix_name() == joint_name &&
                   interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
          });

      if (position_handle == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor state handle for %s", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      const auto velocity_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&joint_name](const auto &interface)
          {
            return interface.get_prefix_name() == joint_name &&
                   interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
          });

      if (velocity_handle == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor state handle for %s", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      const auto effort_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&joint_name](const auto &interface)
          {
            return interface.get_prefix_name() == joint_name &&
                   interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
          });

      if (effort_handle == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor state handle for %s", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      const auto command_handle = std::find_if(
          command_interfaces_.begin(), command_interfaces_.end(),
          [&joint_name](const auto &interface)
          {
            return interface.get_prefix_name() == joint_name &&
                   interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
          });

      if (command_handle == command_interfaces_.end())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor command handle for %s", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      registered_handles.emplace_back(
          JointHandle{std::ref(*position_handle), std::ref(*velocity_handle), std::ref(*effort_handle), std::ref(*command_handle)});
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  controller_interface::CallbackReturn OrthrusControllerBase::configure_imu(
      const std::vector<std::string> &imu_data_types,
      const std::vector<std::string> &imu_names,
      std::vector<ImuHandle> &registered_handles)
  {
    auto logger = get_node()->get_logger();

    if (imu_names.empty())
    {
      RCLCPP_ERROR(logger, "No motor names specified");
      return controller_interface::CallbackReturn::ERROR;
    }

    // register handles
    registered_handles.reserve(imu_names.size());

    for (const auto &imu_name : imu_names)
    {
      std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> angular_velocity;
      std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> linear_acceleration;
      std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> orientation;

      for (const auto &imu_data_type : imu_data_types)
      {
        const auto imu_handle = std::find_if(
            state_interfaces_.cbegin(), state_interfaces_.cend(),
            [&imu_name, &imu_data_type](const auto &interface)
            {
              return interface.get_prefix_name() == imu_name &&
                     interface.get_interface_name() == imu_data_type;
            });

        if (imu_handle == state_interfaces_.cend())
        {
          RCLCPP_ERROR(logger, "Unable to obtain motor state handle for %s", imu_name.c_str());
          return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger, "%s %s", imu_name.c_str(), imu_data_type.c_str());

        if (imu_data_type == "angular_velocity.x" || imu_data_type == "angular_velocity.y" || imu_data_type == "angular_velocity.z")
        {
          // RCLCPP_INFO(get_node()->get_logger(), "imu %lf", *imu_handle.get().get_value());
          angular_velocity.emplace_back(std::ref(*imu_handle));
        }

        if (imu_data_type == "linear_acceleration.x" || imu_data_type == "linear_acceleration.y" || imu_data_type == "linear_acceleration.z")
        {
          linear_acceleration.emplace_back(std::ref(*imu_handle));
        }

        if (imu_data_type == "orientation.w" || imu_data_type == "orientation.x" || imu_data_type == "orientation.y" || imu_data_type == "orientation.z")
        {
          orientation.emplace_back(std::ref(*imu_handle));
        }
      }
      registered_handles.emplace_back(ImuHandle{angular_velocity, linear_acceleration, orientation});
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  controller_interface::CallbackReturn OrthrusControllerBase::configure_leg_imu(
      const std::vector<std::string> &leg_imu_data_types,
      const std::vector<std::string> &leg_imu_names,
      std::vector<LegImuHandle> &registered_handles)
  {
    auto logger = get_node()->get_logger();

    if (leg_imu_names.empty())
    {
      RCLCPP_ERROR(logger, "No motor names specified");
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.reserve(leg_imu_names.size());

    for (const auto &leg_imu_name : leg_imu_names)
    {
      std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> orientation;
      for (const auto &leg_imu_data_type : leg_imu_data_types)
      {
        const auto imu_handle = std::find_if(
            state_interfaces_.cbegin(), state_interfaces_.cend(),
            [&leg_imu_name, &leg_imu_data_type](const auto &interface)
            {
              return interface.get_prefix_name() == leg_imu_name &&
                     interface.get_interface_name() == leg_imu_data_type;
            });

        RCLCPP_INFO(logger, "%s %s", leg_imu_name.c_str(), leg_imu_data_type.c_str());

        if (leg_imu_data_type == "orientation.w" || leg_imu_data_type == "orientation.x" || leg_imu_data_type == "orientation.y" || leg_imu_data_type == "orientation.z")
        {
          orientation.emplace_back(std::ref(*imu_handle));
        }
      }
      registered_handles.emplace_back(LegImuHandle{orientation});
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  controller_interface::CallbackReturn OrthrusControllerBase::configure_flag(
      const std::vector<std::string> &flag_data_types,
      const std::vector<std::string> &flag_names,
      std::vector<FlagHandle> &registered_handles)
  {
    auto logger = get_node()->get_logger();

    for (const auto &flag_name : flag_names)
    {
      const auto enable_power_command_handle = std::find_if(
          command_interfaces_.begin(), command_interfaces_.end(),
          [&flag_name](const auto &interface)
          {
            return interface.get_prefix_name() == flag_name &&
                   interface.get_interface_name() == "enable_power";
          });

      if (enable_power_command_handle == command_interfaces_.end())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor command handle for %s", flag_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      //--
      const auto calibration_position_handle = std::find_if(
          command_interfaces_.begin(), command_interfaces_.end(),
          [&flag_name](const auto &interface)
          {
            return interface.get_prefix_name() == flag_name &&
                   interface.get_interface_name() == "calibration_position";
          });

      if (calibration_position_handle == command_interfaces_.end())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor command handle for %s", flag_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      //--
      const auto calibration_encoder_handle = std::find_if(
          command_interfaces_.begin(), command_interfaces_.end(),
          [&flag_name](const auto &interface)
          {
            return interface.get_prefix_name() == flag_name &&
                   interface.get_interface_name() == "calibration_encoder";
          });

      if (calibration_encoder_handle == command_interfaces_.end())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor command handle for %s", flag_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      registered_handles.emplace_back(FlagHandle{std::ref(*enable_power_command_handle), std::ref(*calibration_position_handle), std::ref(*calibration_encoder_handle)});
    }
  }
}
