// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#include "pantograph_mimick_controller/pantograph_mimick_controller.hpp"

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

const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

PantographMimickController::PantographMimickController()
: controller_interface::ControllerInterface(), pantograph_model_()
{
}

controller_interface::CallbackReturn PantographMimickController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PantographMimickController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  pantograph_joint_names_ = {
    params_.prefix + "panto_a1",
    params_.prefix + "panto_a2",
    params_.prefix + "panto_a3",
    params_.prefix + "panto_a4",
    params_.prefix + "panto_a5",
    params_.prefix + "tool_theta_joint",
    params_.prefix + "tool_phi_joint",
    params_.prefix + "needle_interaction_joint"};

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PantographMimickController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.push_back(
    pantograph_joint_names_[1] + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[2] + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[3] + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[5] + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[6] + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[7] + "/" + HW_IF_POSITION);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration PantographMimickController::
state_interface_configuration()
const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn PantographMimickController::on_activate(
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

controller_interface::CallbackReturn PantographMimickController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend()) {
    return interface_and_value->second;
  } else {
    return kUninitializedValue;
  }
}

controller_interface::return_type PantographMimickController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (param_listener_->is_old(params_)) {
    if (read_parameters() != controller_interface::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to read parameters during update");
      return controller_interface::return_type::ERROR;
    }
  }

  for (const auto & state_interface : state_interfaces_) {
    name_if_value_mapping_[
      state_interface.get_prefix_name()][state_interface.get_interface_name()] =
      state_interface.get_value();
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s/%s: %f\n", state_interface.get_prefix_name().c_str(),
      state_interface.get_interface_name().c_str(), state_interface.get_value());
  }

  // Get the current (active) joint positions
  Eigen::Vector<double, 2> q;

  double pos_a1 = get_value(name_if_value_mapping_, pantograph_joint_names_[0], HW_IF_POSITION);
  double pos_a5 = get_value(name_if_value_mapping_, pantograph_joint_names_[4], HW_IF_POSITION);
  q << pos_a1, pos_a5;

  // Calculate the pantograph (passive) joint positions
  Eigen::Vector<double, 5> panto_joint_state;
  panto_joint_state = pantograph_model_.populate_all_joint_positions(q);

  // Calculate the passive joints of the complete system
  Eigen::Vector<double, 8> full_joint_state;
  full_joint_state = pantograph_model_.populate_all_joint_positions_full_system(q);

  double pos_a2 = panto_joint_state[1];
  double pos_a3 = -pos_a2 - pos_a1;
  double pos_a4 = panto_joint_state[3];

  double pos_theta = full_joint_state[5];
  double pos_phi = full_joint_state[6];
  double pos_needle = full_joint_state[7];

  // write mimics to HW
  command_interfaces_[0].set_value(pos_a2);
  command_interfaces_[1].set_value(pos_a3);
  command_interfaces_[2].set_value(pos_a4);

  // Write angles of the end effector universal joint
  command_interfaces_[3].set_value(pos_theta);
  command_interfaces_[4].set_value(pos_phi);

  // Write needle position along insertion axis
  command_interfaces_[5].set_value(pos_needle);

  return controller_interface::return_type::OK;
}


controller_interface::CallbackReturn
PantographMimickController::read_parameters()
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  // Update pantograph model parameters
  pantograph_model_.set_link_lenghts(
    params_.model_parameters.l_a1,
    params_.model_parameters.l_a2,
    params_.model_parameters.l_a3,
    params_.model_parameters.l_a4,
    params_.model_parameters.l_a5);

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace pantograph_mimick_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  pantograph_mimick_controller::PantographMimickController,
  controller_interface::ControllerInterface)
