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

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "pantograph_library/pantograph_model.hpp"

namespace pantograph_library
{

PantographModel::PantographModel()
{
  set_link_lenghts();
}

bool
PantographModel::set_link_lenghts(
  double l_a1,
  double l_a2,
  double l_a3,
  double l_a4,
  double l_a5)
{
  if (l_a1 <= 0 || l_a2 <= 0 || l_a3 <= 0 || l_a4 <= 0 || l_a5 <= 0)
  {
    return false;
  }
  l_a1_ = l_a1;
  l_a2_ = l_a2;
  l_a3_ = l_a3;
  l_a4_ = l_a4;
  l_a5_ = l_a5;
  return true;
}


Eigen::Vector<double, 5>
PantographModel::populate_all_joint_positions(Eigen::Vector<double, 2> q)
{
  auto p = fk(q);
  return ik(p);
}

Eigen::Vector<double, 2>
PantographModel::fk(Eigen::Vector<double, 2> q)
{
  // Compute position at A2 and A4
  Eigen::Vector2d p_at_A2, p_at_A4;
  p_at_A2[0] = l_a1_ * std::cos(q[0]);
  p_at_A2[1] = l_a1_ * std::sin(q[0]);
  p_at_A4[0] = l_a4_ * std::cos(q[1]) - l_a5_;
  p_at_A4[1] = l_a4_ * std::sin(q[1]);

  // Coordinates of P3 with respect to P1
  double dist_A2_to_A4 = (p_at_A2 - p_at_A4).norm();
  double P2Ph =
    (std::pow(
      l_a2_,
      2) - std::pow(l_a3_, 2) + std::pow(dist_A2_to_A4, 2)) / (2 * dist_A2_to_A4);
  Eigen::Vector2d Ph = p_at_A2 + (P2Ph / dist_A2_to_A4) * (p_at_A4 - p_at_A2);
  double P3Ph = std::sqrt(l_a2_ * 2 - P2Ph * 2);
  Eigen::Vector2d p_at_A3;
  p_at_A3[0] = Ph[0] + (P3Ph / dist_A2_to_A4) * (p_at_A4[1] - p_at_A2[1]);
  p_at_A3[1] = Ph[1] - (P3Ph / dist_A2_to_A4) * (p_at_A4[0] - p_at_A2[0]);

  return p_at_A3;
}

Eigen::Vector<double, 5>
PantographModel::ik(Eigen::Vector<double, 2> p)
{
  Eigen::Vector<double, 5> jnt_pos;

  double a13 = p.norm();
  double a53 = std::sqrt(std::pow(p[0] + l_a5_, 2) + std::pow(p[1], 2));

  double alpha1 =
    std::acos((std::pow(l_a1_, 2) + std::pow(a13, 2) - std::pow(l_a2_, 2)) / (2 * l_a1_ * a13));
  double beta1 = std::atan2(p[1], -p[0]);
  jnt_pos[0] = PI_CST - alpha1 - beta1;

  double alpha5 = std::atan2(p[1], p[0] + l_a5_);
  double beta5 =
    std::acos((std::pow(a53, 2) + std::pow(l_a4_, 2) - std::pow(l_a3_, 2)) / (2 * a53 * l_a4_));
  jnt_pos[4] = alpha5 + beta5;

  // Finding the angles theta2 and theta3 for the simulation
  double beta2 =
    std::acos((std::pow(l_a2_, 2) + std::pow(a13, 2) - std::pow(l_a1_, 2)) / (2 * l_a2_ * a13));
  double beta3 =
    std::acos((std::pow(l_a3_, 2) + std::pow(a53, 2) - std::pow(l_a4_, 2)) / (2 * l_a3_ * a53));
  double beta4 = PI_CST - beta3 - beta5;

  double alpha2 = PI_CST - alpha1 - beta2;
  double alpha3 = PI_CST - alpha5 - beta1;

  jnt_pos[1] = PI_CST - alpha2;
  jnt_pos[2] = PI_CST - beta2 - alpha3 - beta3;
  jnt_pos[3] = -(PI_CST - beta4);

  return jnt_pos;
}

Eigen::Matrix<double, 2, 2>
PantographModel::jacobian(Eigen::Vector<double, 2> q, Eigen::Vector<double, 2> q_dot)
{
  (void) q;
  (void) q_dot;
  // TODO
  throw std::logic_error("Function not yet implemented!");
  return Eigen::Matrix<double, 2, 2>::Zero();
}

}  // namespace pantograph_library
