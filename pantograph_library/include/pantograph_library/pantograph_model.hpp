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

#ifndef PANTOGRAPH_LIBRARY__PANTOGRAPH_MODEL_HPP_
#define PANTOGRAPH_LIBRARY__PANTOGRAPH_MODEL_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace pantograph_library
{

constexpr double PI_CST = 3.14159265358979323846;

class PantographModel
{
public:
  PantographModel();

  /// Set link lengths used by the models
  bool set_link_lenghts(
    double l_a1 = 0.1,
    double l_a2 = 0.165,
    double l_a3 = 0.165,
    double l_a4 = 0.1,
    double l_a5 = 0.085);  // length in m

  /// Populate all joints position
  /// -> [jnt_a1, jnt_a2, ..., jnt_a5] = populate_all_joint_positions(q = [jnt_a1, jnt_a5])
  Eigen::Vector<double, 5> populate_all_joint_positions(Eigen::Vector<double, 2> q);

  /// Forward kinematics p = fk([q1, q2]) = fk([joint_a1, joint_a5])
  Eigen::Vector<double, 2> fk(Eigen::Vector<double, 2> q);

  /// Inverse kinematics [joint_a1, ..., joint_a5] = ik(p)
  Eigen::Vector<double, 5> ik(Eigen::Vector<double, 2> p);

  /// Jacobian J(q, q_dot), where q = [joint_a1, joint_a5].T
  Eigen::Matrix<double, 2, 2> jacobian(Eigen::Vector<double, 2> q, Eigen::Vector<double, 2> q_dot);

  /// Forward kinematics of the complete system p = fk_needle(P3) = fk_needle(fk([q1,q2]))
  Eigen::Vector<double, 3> fk_system(Eigen::Vector<double, 2> q);

  /// Inverse kinematics of the complete system p = fk_needle(P3) = fk_needle(fk([q1,q2]))
  Eigen::Vector<double, 8> ik_system(Eigen::Vector<double, 3> p);

  /// Inverse kinematics of the pantograph according to equations of T.CESARE report
  Eigen::Vector<double, 5> ik_panto_optimized(Eigen::Vector<double, 2> p);

  /// Populate all joints position of the full system (pantograph + needle)
  Eigen::Vector<double, 8> populate_all_joint_positions_full_system(Eigen::Vector<double, 2> q);

  /// Calculate the force that the pantograph needs to apply to create a force felt by the user
  double get_panto_force(Eigen::Vector<double, 3> p, double f_guide, double alpha);

protected:
  /// First left link length in m (i.e., A1->A2)
  double l_a1_;

  /// Second left link length in m (i.e., A2->A3)
  double l_a2_;

  /// First right link length in m (i.e., A5->A4)
  double l_a4_;

  /// Second right link length in m (i.e., A4->A3)
  double l_a3_;

  /// Dist between A1 and A5 in m (i.e., A4->A5)
  double l_a5_;

  /// Length of the needle in m
  double l_needle_ = 0.2;  //  in m

  /// Coords of insertion point according to CAD
  double PI_x = 0.0425;
  double PI_y = 0.16056;
  double PI_z = 0.09;
};

}  // namespace pantograph_library

#endif  // PANTOGRAPH_LIBRARY__PANTOGRAPH_MODEL_HPP_
