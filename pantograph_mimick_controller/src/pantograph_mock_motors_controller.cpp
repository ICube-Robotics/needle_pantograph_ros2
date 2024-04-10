// Copyright 2023, ICube Laboratory, University of Strasbourg
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

#include "pantograph_mimick_controller/pantograph_mock_motors_controller.hpp"
#include "pantograph_controller_utils.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"


namespace pantograph_mimick_controller
{

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_ACCELERATION;

PantographMockMotorsController::PantographMockMotorsController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn PantographMockMotorsController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PantographMockMotorsController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string prefix = "";
  pantograph_joint_names_ = {
    prefix + "panto_a1",
    prefix + "panto_a2",
    prefix + "panto_a3",
    prefix + "panto_a4",
    prefix + "panto_a5"
  };

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PantographMockMotorsController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.push_back(
    pantograph_joint_names_[0] + "/" + HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[4] + "/" + HW_IF_VELOCITY);

  command_interfaces_config.names.push_back(
    pantograph_joint_names_[0] + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[4] + "/" + HW_IF_POSITION);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration PantographMockMotorsController::
state_interface_configuration()
const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn PantographMockMotorsController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
    command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PantographMockMotorsController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PantographMockMotorsController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (const auto & state_interface : state_interfaces_) {
    name_if_value_mapping_[
      state_interface.get_prefix_name()][state_interface.get_interface_name()] =
      state_interface.get_value();
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s/%s: %f\n", state_interface.get_prefix_name().c_str(),
      state_interface.get_interface_name().c_str(), state_interface.get_value());
  }

  // Get the previous (active) joint torques
  double torque_a1 = get_value(name_if_value_mapping_, pantograph_joint_names_[0], HW_IF_EFFORT);
  double torque_a5 = get_value(name_if_value_mapping_, pantograph_joint_names_[4], HW_IF_EFFORT);

  // Get external torques
  double tau_ext_a1 = get_value(name_if_value_mapping_, "fake_tau_ext_1", HW_IF_EFFORT);
  double tau_ext_a5 = get_value(name_if_value_mapping_, "fake_tau_ext_5", HW_IF_EFFORT);

  // Calculate the pantograph (passive) joint acc. from torques
  double equivalent_joint_inertia = 0.02;  // kg m^2
  double last_acc_a1 = 1 / equivalent_joint_inertia * (torque_a1 + tau_ext_a1);
  double last_acc_a5 = 1 / equivalent_joint_inertia * (torque_a5 + tau_ext_a5);

  // Get the previous (active) joint positions and velocities
  double last_pos_a1 =
    get_value(name_if_value_mapping_, pantograph_joint_names_[0], HW_IF_POSITION);
  double last_vel_a1 =
    get_value(name_if_value_mapping_, pantograph_joint_names_[0], HW_IF_VELOCITY);
  double last_pos_a5 =
    get_value(name_if_value_mapping_, pantograph_joint_names_[4], HW_IF_POSITION);
  double last_vel_a5 =
    get_value(name_if_value_mapping_, pantograph_joint_names_[4], HW_IF_VELOCITY);

  // Integrate state variables
  double dt = period.seconds();
  double vel_a1 = last_vel_a1 + last_acc_a1 * dt;
  double pos_a1 = last_pos_a1 + vel_a1 * dt;

  double vel_a5 = last_vel_a5 + last_acc_a5 * dt;
  double pos_a5 = last_pos_a5 + vel_a5 * dt;

  // write mimics to HW
  command_interfaces_[0].set_value(vel_a1);
  command_interfaces_[1].set_value(vel_a5);

  command_interfaces_[2].set_value(pos_a1);
  command_interfaces_[3].set_value(pos_a5);

  return controller_interface::return_type::OK;
}

}  // namespace pantograph_mimick_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  pantograph_mimick_controller::PantographMockMotorsController,
  controller_interface::ControllerInterface)
