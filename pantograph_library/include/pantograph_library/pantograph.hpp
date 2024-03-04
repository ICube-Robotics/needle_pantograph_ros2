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

#ifndef PANTOGRAPH_LIBRARY__PANTOGRAPH_HPP_
#define PANTOGRAPH_LIBRARY__PANTOGRAPH_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace pantograph
{
constexpr double PI_CST = 3.14159265358979323846;

class PantographModel
{
public:
  PantographModel() {}

  /// Populate all joints position
  /// -> [jnt_a1, jnt_a2, ..., jnt_a5] = populate_all_joint_positions(q = [jnt_a1, jnt_a5])
  Eigen::Vector<double, 5> populate_all_joint_positions(Eigen::Vector<double, 2> q)
  {
    auto p = fk(q);
    return ik(p);
  }

  /// Forward kinematics p = fk([q1, q2]) = fk([joint_a1, joint_a5])
  Eigen::Vector<double, 2> fk(Eigen::Vector<double, 2> q)
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
    p_at_A3[0] = Ph[0] + (P3Ph / dist_A2_to_A4) * (P4[1] - P2[1]);
    p_at_A3[1] = Ph[1] - (P3Ph / dist_A2_to_A4) * (P4[0] - P2[0]);

    return p_at_A3;
  }

  /// Inverse kinematics [joint_a1, ..., joint_a5] = ik(p)
  Eigen::Vector<double, 5> ik(Eigen::Vector<double, 2> p)
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
      std::acos((std::pow(a53, 2) + std::pow(a4, 2) - std::pow(a3, 2)) / (2 * a53 * l_a4_));
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

  /// Jacobian J(q, q_dot), where q = [joint_a1, joint_a5].T
  Eigen::Matrix<double, 2, 2> jacobian(Eigen::Vector<double, 2> q, Eigen::Vector<double, 2> q_dot)
  {
    return Eigen::Matrix<double, 2, 2>::Zero();
  }

protected:
  /// First left link length in m (i.e., A1->A2)
  double l_a1_ = 0.1;

  /// Second left link length in m (i.e., A2->A3)
  double l_a2_ = 0.165;

  /// First right link length in m (i.e., A5->A4)
  double l_a4_ = 0.1;

  /// Second right link length in m (i.e., A4->A3)
  double l_a3_ = 0.165;

  /// Dist between A1 and A5 in m (i.e., A4->A5)
  double l_a5_ = 0.085;
}

}  // namespace pantograph

#endif  // PANTOGRAPH_LIBRARY__PANTOGRAPH_HPP_
