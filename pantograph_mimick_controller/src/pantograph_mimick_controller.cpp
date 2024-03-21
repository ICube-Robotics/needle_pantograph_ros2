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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace pantograph_mimick_controller
{

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

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PantographMimickController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.push_back(
    params_.prefix + "panto_a2" + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(
    params_.prefix + "panto_a3" + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(
    params_.prefix + "panto_a4" + "/" + HW_IF_POSITION);

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

controller_interface::return_type PantographMimickController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto index = 0ul; index < command_interfaces_.size(); ++index) {
    command_interfaces_[index].set_value(0.0);
  }

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

  if (params_.model.link_lengths.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'model.link_lengths' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Update pantograph model parameters
  pantograph_model_.set_link_lenghts(
    params_.model.link_lengths[0],
    params_.model.link_lengths[1],
    params_.model.link_lengths[2],
    params_.model.link_lengths[3],
    params_.model.link_lengths[4]);

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace pantograph_mimick_controller
