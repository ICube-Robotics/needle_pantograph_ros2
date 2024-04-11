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

#include "pantograph_mimick_controller/pantograph_mock_operator_controller.hpp"
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

PantographMockOperatorController::PantographMockOperatorController()
: controller_interface::ControllerInterface(), pantograph_model_()
{
}

controller_interface::CallbackReturn PantographMockOperatorController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PantographMockOperatorController::on_configure(
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

  fake_sensors_joint_names_ = {
    prefix + "fake_tau_ext_1",
    prefix + "fake_tau_ext_5"
  };


  joints_command_subscriber_ = \
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/fake_operator_position",    // "~/operator_desired_position",
    rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {rt_command_ptr_.writeFromNonRT(msg);});

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PantographMockOperatorController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.push_back(
    fake_sensors_joint_names_[0] + "/" + HW_IF_EFFORT);
  command_interfaces_config.names.push_back(
    fake_sensors_joint_names_[1] + "/" + HW_IF_EFFORT);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration PantographMockOperatorController::
state_interface_configuration()
const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn PantographMockOperatorController::on_activate(
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


  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = \
    realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PantographMockOperatorController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PantographMockOperatorController::update(
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


  auto operator_position_msg = rt_command_ptr_.readFromRT();
  // no command received yet
  if (!operator_position_msg || !(*operator_position_msg)) {
    command_interfaces_[0].set_value(0.0);
    command_interfaces_[1].set_value(0.0);
    return controller_interface::return_type::OK;
  }

  Eigen::Vector2d ph = Eigen::Vector2d(
    (*operator_position_msg)->data[0],
    (*operator_position_msg)->data[1]
  );

  // Get the (active) joint positions and velocities
  double pos_a1 =
    get_value(name_if_value_mapping_, pantograph_joint_names_[0], HW_IF_POSITION);
  double vel_a1 =
    get_value(name_if_value_mapping_, pantograph_joint_names_[0], HW_IF_VELOCITY);
  double pos_a5 =
    get_value(name_if_value_mapping_, pantograph_joint_names_[4], HW_IF_POSITION);
  double vel_a5 =
    get_value(name_if_value_mapping_, pantograph_joint_names_[4], HW_IF_VELOCITY);

  Eigen::Vector2d q = Eigen::Vector2d(pos_a1, pos_a5);
  Eigen::Vector2d q_dot = Eigen::Vector2d(vel_a1, vel_a5);
  auto J = pantograph_model_.jacobian(q);
  Eigen::Vector2d p = pantograph_model_.fk(q);
  Eigen::Vector2d p_dot = J.transpose() * q_dot;
  double cst = 9;
  auto tau = J.transpose().inverse() * (
    cst * (ph - p) - 2 * 0.9 * sqrt(cst) * p_dot
  );
  /*
  std::cout << "\n q = " << q.transpose() << std::endl;
  std::cout << "P = " << p.transpose() << std::endl;
  std::cout << "P_h = " << ph.transpose() << std::endl;
  std::cout << "err P = " << (ph - p).transpose() << std::endl;
  std::cout << "p_dot = " << p_dot.transpose() << std::endl;
  std::cout << "J = \n" << J << std::endl;
  std::cout << "Fc = " << Fc.transpose() << std::endl;
  std::cout << "tau = " << tau.transpose() << std::endl;
  */

  // write mimics to HW
  command_interfaces_[0].set_value(tau(0));
  command_interfaces_[1].set_value(tau(1));
  return controller_interface::return_type::OK;
}

}  // namespace pantograph_mimick_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  pantograph_mimick_controller::PantographMockOperatorController,
  controller_interface::ControllerInterface)
