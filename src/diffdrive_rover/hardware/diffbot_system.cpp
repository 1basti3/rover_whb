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

#include "include/diffdrive_rover/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_rover
{
hardware_interface::CallbackReturn DiffDriveRoverHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.drivewhl_l_f = info_.hardware_parameters["drivewhl_l_f_name"];
  cfg_.drivewhl_l_r = info_.hardware_parameters["drivewhl_l_r_name"];
  cfg_.drivewhl_r_f = info_.hardware_parameters["drivewhl_r_f_name"];
  cfg_.drivewhl_r_r = info_.hardware_parameters["drivewhl_r_r_name"];
  cfg_.device = info_.hardware_parameters["device"];
  //cfg_.loop_rate = stof(info_.hardware_parameters["loop_rate"]);
  cfg_.baud_rate = stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  drivewhl_l_f.setup(cfg_.drivewhl_l_f, cfg_.enc_counts_per_rev);
  drivewhl_l_r.setup(cfg_.drivewhl_l_r, cfg_.enc_counts_per_rev);
  drivewhl_r_f.setup(cfg_.drivewhl_r_f, cfg_.enc_counts_per_rev);
  drivewhl_r_r.setup(cfg_.drivewhl_r_r, cfg_.enc_counts_per_rev);


  
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveRoverHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveRoverHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveRoverHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveRoverHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveRoverHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveRoverHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces; 

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      drivewhl_l_f.name, hardware_interface::HW_IF_POSITION, &drivewhl_l_f.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      drivewhl_l_f.name, hardware_interface::HW_IF_VELOCITY, &drivewhl_l_f.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      drivewhl_l_r.name, hardware_interface::HW_IF_POSITION, &drivewhl_l_r.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      drivewhl_l_r.name, hardware_interface::HW_IF_VELOCITY, &drivewhl_l_r.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      drivewhl_r_f.name, hardware_interface::HW_IF_POSITION, &drivewhl_r_f.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      drivewhl_r_f.name, hardware_interface::HW_IF_VELOCITY, &drivewhl_r_f.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      drivewhl_r_r.name, hardware_interface::HW_IF_POSITION, &drivewhl_r_r.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      drivewhl_r_r.name, hardware_interface::HW_IF_VELOCITY, &drivewhl_r_r.vel));
  

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveRoverHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces; 

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      drivewhl_l_f.name, hardware_interface::HW_IF_VELOCITY, &drivewhl_l_f.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      drivewhl_l_r.name, hardware_interface::HW_IF_VELOCITY, &drivewhl_l_f.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      drivewhl_r_f.name, hardware_interface::HW_IF_VELOCITY, &drivewhl_r_f.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      drivewhl_r_r.name, hardware_interface::HW_IF_VELOCITY, &drivewhl_r_f.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveRoverHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoverHardware"), "Activating ...please wait...");

  comms_.connect();//cfg_.device, cfg_.timeout_ms);

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoverHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveRoverHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoverHardware"), "Deactivating ...please wait...");

  comms_.disconnect();

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoverHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveRoverHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  comms_.read_left_encoder(drivewhl_l_f.enc);
  comms_.read_right_encoder(drivewhl_r_f.enc);

  double delta_seconds = period.seconds();

  double pos_prev_l = drivewhl_l_f.pos;
  drivewhl_l_f.pos = drivewhl_l_f.calc_enc_angle();
  drivewhl_l_f.vel = (drivewhl_l_f.pos - pos_prev_l) / delta_seconds;

  drivewhl_l_r.pos = drivewhl_l_f.pos;
  drivewhl_l_r.vel = drivewhl_l_f.vel;

  double pos_prev_r = drivewhl_r_f.pos;
  drivewhl_r_f.pos = drivewhl_r_f.calc_enc_angle();
  drivewhl_r_f.vel = (drivewhl_r_f.pos - pos_prev_r) / delta_seconds;

  drivewhl_r_r.pos = drivewhl_r_f.pos;
  drivewhl_r_r.vel = drivewhl_r_f.vel;


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_rover ::DiffDriveRoverHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  comms_.set_motor_values(drivewhl_l_f.cmd, drivewhl_r_f.cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_rover

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_rover::DiffDriveRoverHardware, hardware_interface::SystemInterface)
