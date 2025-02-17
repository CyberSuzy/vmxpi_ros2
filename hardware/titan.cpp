// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vmxpi_ros2/titan.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vmxpi_ros2
{
hardware_interface::CallbackReturn TitanSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.DiffBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // *** Hardware initialization code here using your Titan driver ***
  // Initialize Titan driver in on_init, as a member variable
  try {
    titan_driver_ = std::make_unique<studica_driver::Titan>(
        "titan_controller", 45, 15600, 0.0006830601, 0.8 // Example parameters, adjust as needed
    );
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "Error initializing Titan driver: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Titan driver initialized in on_init"); // Add log message for clarity.

  // Initialize state and command vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TitanSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TitanSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn TitanSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!titan_driver_) { // Safety check - should not happen if on_init is successful
    RCLCPP_ERROR(get_logger(), "Titan driver is not initialized in on_activate!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  titan.ConfigureEncoder(0, 0.0006830601); //1464 = 1 rotation
  titan.ConfigureEncoder(1, 0.0006830601);
  titan.ConfigureEncoder(2, 0.0006830601);
  titan.ConfigureEncoder(3, 0.0006830601);
  
  titan.ResetEncoder(0);
  titan.ResetEncoder(1);
  titan.ResetEncoder(2);
  titan.ResetEncoder(3);

    // These are flags to be uncommented as needed
  // titan.InvertEncoderDirection(0);
  // titan.InvertEncoderDirection(1);
  // titan.InvertEncoderDirection(2);
  // titan.InvertEncoderDirection(3);

  // titan.InvertMotorDirection(0);
  // titan.InvertMotorDirection(1);
  // titan.InvertMotorDirection(2);
  // titan.InvertMotorDirection(3);

  // titan.InvertMotorRPM(0);
  // titan.InvertMotorRPM(1);
  // titan.InvertMotorRPM(2);
  // titan.InvertMotorRPM(3);

  titan.Enable(true);

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TitanSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (titan_driver_) { // Check if titan_driver_ is valid before using
    titan_driver_->Enable(false);
    std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(5.0 * 1000))); //This Delay is just to check if the Titan does disable
  } else {
    RCLCPP_WARN(get_logger(), "Titan driver is not initialized in on_deactivate, nothing to disable.");
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TitanSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (titan_driver_) {

    for (std::size_t i = 0; i < hw_velocities_.size(); i++) // Loop through joints
    {
      // For wheel_left_joint (motor 0), wheel_right_joint (motor 1) - **VERIFY JOINT-MOTOR MAPPING**
      if (i == 0) { // Assuming joint index 0 is left wheel, motor 0
          hw_positions_[i] = static_cast<double>(titan_driver_->GetEncoderCount(0)); // Raw encoder counts as position
          hw_velocities_[i] = static_cast<double>(titan_driver_->GetRPM(0)); // RPM as velocity
      } else if (i == 1) { // Assuming joint index 1 is right wheel, motor 1
          hw_positions_[i] = static_cast<double>(titan_driver_->GetEncoderCount(1)); // Raw encoder counts as position
          hw_velocities_[i] = static_cast<double>(titan_driver_->GetRPM(1)); // RPM as velocity
      } else { // Handle cases if you have more joints - maybe log a warning or error if unexpected
          RCLCPP_WARN(get_logger(), "Unexpected joint index %zu in read() - Assuming only 2 wheels for now.", i);
      }
    }
    // RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "%s", ss_read.str().c_str()); // Use RCLCPP_DEBUG or RCLCPP_INFO_THROTTLE for less verbose output
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Encoder Counts: Left: %f, Right: %f, RPM: Left: %f, Right: %f", // Less verbose logging
                        hw_positions_[0], hw_positions_[1],
                        hw_velocities_[0], hw_velocities_[1]);

  } else {
      RCLCPP_ERROR(get_logger(), "Titan driver is not initialized in read()!"); // Important error log
      return hardware_interface::return_type::ERROR; // Return error to signal issue
  }
return hardware_interface::return_type::OK;
}

hardware_interface::return_type vmxpi_ros2 ::TitanSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (titan_driver_) {
    for (std::size_t i = 0; i < hw_commands_.size(); i++)
    {
      // For wheel_left_joint (motor 0), wheel_right_joint (motor 1) - **VERIFY JOINT-MOTOR MAPPING**
      if (i == 0) { // Assuming joint index 0 is left wheel, motor 0
          titan_driver_->SetSpeed(0, hw_commands_[i]); // Send commanded velocity to motor 0
      } else if (i == 1) { // Assuming joint index 1 is right wheel, motor 1
          titan_driver_->SetSpeed(1, hw_commands_[i]); // Send commanded velocity to motor 1
      } else { // Handle cases if you have more joints - maybe log a warning or error if unexpected
          RCLCPP_WARN(get_logger(), "Unexpected joint index %zu in write() - Assuming only 2 wheels for now.", i);
      }
    }
    // RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "%s", ss_write.str().c_str()); // Use RCLCPP_DEBUG or RCLCPP_INFO_THROTTLE
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Motor Speeds: Left: %f, Right: %f", // Less verbose logging
                         hw_commands_[0], hw_commands_[1]);

} else {
    RCLCPP_ERROR(get_logger(), "Titan driver is not initialized in write()!"); // Important error log
    return hardware_interface::return_type::ERROR; // Return error to signal issue
}

  return hardware_interface::return_type::OK;
}

}// namespace vmxpi_ros2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vmxpi_ros2::TitanSystemHardware, hardware_interface::SystemInterface)
