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

#include "vmxpi_ros2/titan_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <math.h>

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

  titan_driver_->ConfigureEncoder(0, 0.0006830601); //1464 = 1 rotation
  titan_driver_->ConfigureEncoder(1, 0.0006830601);
  titan_driver_->ConfigureEncoder(2, 0.0006830601);
  titan_driver_->ConfigureEncoder(3, 0.0006830601);
  
  titan_driver_->ResetEncoder(0);
  titan_driver_->ResetEncoder(1);
  titan_driver_->ResetEncoder(2);
  titan_driver_->ResetEncoder(3);

    // These are flags to be uncommented as needed
  titan_driver_->InvertEncoderDirection(0);
  titan_driver_->InvertEncoderDirection(1);
  // titan_driver_->InvertEncoderDirection(2);
  // titan_driver_->InvertEncoderDirection(3);

  titan_driver_->InvertMotorDirection(0);
  titan_driver_->InvertMotorDirection(1);
  // titan_driver_->InvertMotorDirection(2);
  // titan_driver_->InvertMotorDirection(3);

  titan_driver_->InvertMotorRPM(0);
  titan_driver_->InvertMotorRPM(1);
  // titan_driver_->InvertMotorRPM(2);
  // titan_driver_->InvertMotorRPM(3);

  titan_driver_->Enable(true);

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

  RCLCPP_INFO(get_logger(), "Titan Successfully activated!");

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

  RCLCPP_INFO(get_logger(), "Titan Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TitanSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (titan_driver_) {
    double period_seconds = period.seconds(); // Get the time in seconds since last read

    static int32_t last_encoder_count_left = 0;  // Static variables to store previous encoder counts
    static int32_t last_encoder_count_right = 0;

    int32_t current_encoder_count_left = (titan_driver_->GetEncoderCount(2)+titan_driver_->GetEncoderCount(3))/2;
    int32_t current_encoder_count_right = (titan_driver_->GetEncoderCount(0)+titan_driver_->GetEncoderCount(1))/2;

    // --- Velocity Calculation (based on encoder count difference over time) ---
    // ** IMPORTANT: You will likely need to adjust the scaling factor! **
    double encoder_ticks_per_revolution = 1464.0; // Example: Adjust to your encoder's ticks per revolution
    double wheel_circumference = M_PI * 0.1; // Example: Wheel circumference in meters (adjust to your robot)

    double delta_encoder_count_left = current_encoder_count_left - last_encoder_count_left;
    double delta_encoder_count_right = current_encoder_count_right - last_encoder_count_right;

    // Calculate distance traveled by each wheel in meters since last read:
    double distance_left = (delta_encoder_count_left / encoder_ticks_per_revolution) * wheel_circumference;
    double distance_right = (delta_encoder_count_right / encoder_ticks_per_revolution) * wheel_circumference;

    // Calculate velocity in meters per second:
    hw_velocities_[0] = distance_left / period_seconds;  // Left wheel velocity (m/s)
    hw_velocities_[1] = distance_right / period_seconds; // Right wheel velocity (m/s)

    // --- Position Update (still using raw encoder counts for position state) ---
    hw_positions_[0] = static_cast<double>(current_encoder_count_left); // Raw encoder counts as position (for state)
    hw_positions_[1] = static_cast<double>(current_encoder_count_right);

    // --- RPM Reading (still keeping RPM reading for velocity state - you can choose either velocity calculation OR RPM) ---
    // If you want to use calculated velocity from encoder counts as the primary velocity state, you might remove or comment out these RPM readings.
    // If you want to keep RPM as a separate velocity state, leave these lines in.
    // hw_velocities_[0] = static_cast<double>(titan_driver_->GetRPM(0)); // RPM as velocity (you could comment this out if using calculated velocity)
    // hw_velocities_[1] = static_cast<double>(titan_driver_->GetRPM(1)); // RPM as velocity (you could comment this out if using calculated velocity)

    // --- Update 'last' encoder counts for the next read cycle ---
    last_encoder_count_left = current_encoder_count_left;
    last_encoder_count_right = current_encoder_count_right;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Motor positions: Left: %f, Right: %f", // Less verbose logging
    hw_positions_[0], hw_positions_[1]);


  } else {
    RCLCPP_ERROR(get_logger(), "Titan driver is not initialized in read()!");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type vmxpi_ros2 ::TitanSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (titan_driver_) {
    for (std::size_t i = 0; i < hw_commands_.size(); i++)
    {
      double speedCfg = hw_commands_[i] * 0.05;
      // For wheel_left_joint (motor 0), wheel_right_joint (motor 1) - **VERIFY JOINT-MOTOR MAPPING**
      if (i == 0) { // Assuming joint index 0 is left wheel, motor 0
        titan_driver_->SetSpeed(2, speedCfg); // Send commanded velocity to motor 0
        titan_driver_->SetSpeed(3, speedCfg); // Send commanded velocity to motor 0
      } else if (i == 1) { // Assuming joint index 1 is right wheel, motor 1
        titan_driver_->SetSpeed(0, speedCfg); // Send commanded velocity to motor 0
        titan_driver_->SetSpeed(1, speedCfg); // Send commanded velocity to motor 0
      } else { // Handle cases if you have more joints - maybe log a warning or error if unexpected
          RCLCPP_WARN(get_logger(), "Unexpected joint index %zu in write() - Assuming only 2 wheels for now.", i);
      }
    }
    // RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "%s", ss_write.str().c_str()); // Use RCLCPP_DEBUG or RCLCPP_INFO_THROTTLE
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Motor Speeds: Left: %f, Right: %f", // Less verbose logging
    //                      hw_commands_[0], hw_commands_[1]);

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
