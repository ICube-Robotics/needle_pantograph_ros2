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

#ifndef PANTOGRAPH_CONTROLLER_UTILS_HPP_
#define PANTOGRAPH_CONTROLLER_UTILS_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <limits>

namespace pantograph_mimick_controller
{

const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name);

}  // namespace pantograph_mimick_controller

#endif  // PANTOGRAPH_CONTROLLER_UTILS_HPP_
