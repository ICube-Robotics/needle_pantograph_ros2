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

#ifndef PANTOGRAPH_MIMICK_CONTROLLER__PANTOGRAPH_MOCK_OPERATOR_CONTROLLER_HPP_
#define PANTOGRAPH_MIMICK_CONTROLLER__PANTOGRAPH_MOCK_OPERATOR_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "pantograph_mimick_controller/visibility_control.h"
#include "pantograph_library/pantograph_model.hpp"

namespace pantograph_mimick_controller
{

class PantographMockOperatorController : public controller_interface::ControllerInterface
{
public:
  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  PantographMockOperatorController();

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  ~PantographMockOperatorController() = default;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<std::string> joint_names_;
  std::string interface_name_;

  std::vector<std::string> command_interface_types_;

protected:
  std::unordered_map<std::string, std::unordered_map<std::string, double>> name_if_value_mapping_;
  std::vector<std::string> pantograph_joint_names_;
  std::vector<std::string> fake_sensors_joint_names_;
  pantograph_library::PantographModel pantograph_model_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>> rt_command_ptr_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joints_command_subscriber_;
};

}  // namespace pantograph_mimick_controller

#endif  // PANTOGRAPH_MIMICK_CONTROLLER__PANTOGRAPH_MOCK_OPERATOR_CONTROLLER_HPP_
