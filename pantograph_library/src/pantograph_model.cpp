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
  if (l_a1 <= 0 || l_a2 <= 0 || l_a3 <= 0 || l_a4 <= 0 || l_a5 <= 0) {
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
  // (void) q;
  // (void) q_dot;
  // TODO(tpoignonec): add jacobian computation
  // throw std::logic_error("Function not yet implemented!");

  // Get the angles of all the pantograph joints
  Eigen::Vector<double, 5> jnt_pos;
  jnt_pos = populate_all_joint_positions(q);

  double theta2 = jnt_pos[1];  //  Check if IKM is correct
  double theta4 = jnt_pos[3];

  // Jacobian coefficients according to T.CESARE report
  double S14S42 = sin(q(0) - theta4) / sin(theta4 - theta2);
  double S24S42 = sin(q(1) - theta4) / sin(theta4 - theta2);

  double J11 = -l_a1_ * (sin(q(1)) + sin(theta2) * S14S42);
  double J12 = l_a4_ * sin(theta2) * S24S42;
  double J21 = l_a1_ * (cos(q(1)) + cos(theta2) * S14S42);
  double J22 = -l_a4_ * cos(theta2) * S24S42;

  Eigen::Matrix2d J;

  J << J11, J21,
    J21, J22;


  return Eigen::Matrix<double, 2, 2>::Zero();
}

Eigen::Vector<double, 3>
PantographModel::fk_system(Eigen::Vector<double, 2> q)
{
  // FKM of the complete system : calculates cartesian position of interaction point PU
  // according to joint space coordinates q
  Eigen::Vector3d PI, P3, P3I, PIU, PU;

  // Coordinates of insertion point PI in base frame according to CAD
  PI[0] = PI_x;
  PI[0] = PI_y;
  PI[0] = PI_z;
  
  // PI[0] = 0.0425;
  // PI[1] = 0.16056;
  // PI[2] = 0.09;

  // Get coords of P3 in base frame according to FKM
  Eigen::Vector2d P3_2D;
  P3_2D = fk(q);

  // Convert P3 in the plane to coords in 3D
  // P3[0] = P3_2D[0];
  // P3[1] = P3_2D[1];
  // P3[2] = 0;
  P3 << P3_2D, 0;

  // Convert coords of PI in base frame to P3 frame
  P3I = PI - P3;

  // Get azimuth (theta) and elevation (phi) angles (in radians) of vector P3PI
  double theta = std::atan2(P3I[1], P3I[0]);
  double phi = std::atan2(P3I[2], std::sqrt(std::pow(P3I[0], 2) + std::pow(P3I[1], 2)));
  double Lin = std::sqrt(std::pow(P3I[0], 2) + std::pow(P3I[1], 2) + std::pow(P3I[2], 2));

  // Calculate translation from PI to PU
  double Lout = l_needle_ - Lin;
  PIU[0] = Lout * std::cos(phi) * std::cos(theta);  //  Coords of PU in PI frame
  PIU[1] = Lout * std::cos(phi) * std::sin(theta);
  PIU[2] = Lout * std::sin(phi);

  // Convert coords of PU in PI frame to coords in base frame
  PU = PI + PIU;

  return PU;
}
Eigen::Vector<double, 7>
PantographModel::ik_system(Eigen::Vector<double, 3> p)
{
  /* Inverse kinematics model of the complete system:
  computes the angles of all the joints according
  to the cartesian position of interaction point PU*/

  Eigen::Vector3d PI, P3, PI3, PIU, PU;

  // Coords of insertion point PI in base frame according to CAD

  PI[0] = PI_x;
  PI[0] = PI_y;
  PI[0] = PI_z;

  // PI[0] = 0.0425;
  // PI[1] = 0.16056;
  // PI[2] = 0.09;

  // Transformation of PU coords in base frame to coords in PI frame
  PIU = PU - PI;

  // Get azimuth (theta) and elevation (phi) angles (in radians) of vector PIPU
  double theta = std::atan2(PIU[1], PIU[0]);
  double phi = std::atan2(PIU[2], std::sqrt(std::pow(PIU[0], 2) + std::pow(PIU[1], 2)));
  double Lout = std::sqrt(std::pow(PIU[0], 2) + std::pow(PIU[1], 2) + std::pow(PIU[2], 2));

  // Get end effector position P3
  double Lin = l_needle_ - Lout;

  // Transformation from I to P3 in I frame
  PI3[0] = Lin * std::cos(phi) * std::cos(theta);
  PI3[1] = Lin * std::cos(phi) * std::sin(theta);
  PI3[2] = Lin * std::sin(phi);

  // Transformation of P3 coords on I frame to coords in base frame
  P3 = PI - PI3;

  Eigen::Vector<double, 2> P3_2D;
  P3_2D[0] = P3[0];
  P3_2D[1] = P3[1];

  // Get angles of the pantograph joints using IKM
  Eigen::Vector<double, 5> jnt_pos;
  jnt_pos = ik(P3_2D);

  // Return pantograph joints angles + needle orientation angles
  Eigen::Vector<double, 7> jnt_ext_pos;

  jnt_ext_pos << jnt_pos, theta, phi;

  return jnt_ext_pos;
}

// Calculate joint positions of the complete system for visualization
Eigen::Vector<double, 7>
PantographModel::populate_all_joint_positions_full_system(Eigen::Vector<double, 2> q)
{
  auto p = fk_system(q);
  return ik_system(p);
}

// Calculate force applied by the pantograph to create the force felt by the user
double 
PantographModel::get_panto_force(Eigen::Vector<double, 3> p, double f_guide, double alpha)
{
 // Get all the robot joint angles IKM
  Eigen::Vector<double, 7> jnt_ext_pos;
  jnt_ext_pos = ik_system(p); 
  double theta = jnt_ext_pos[5];  // Azimuth angle
  double phi = jnt_ext_pos[6];  // Elevation angle 

  // get insertion point height 
  double H = PI_z; // = 0.09 based on insertion point coords 

  // Calculate length of the needle segment between P3 and PI
  double l_in = H/std::sin(phi);

  // Calculate length of the needle segment between PI and PU
  double l_out = l_needle_ - l_in;

  /*Calculate the perpendicular component of the force that the system 
  needs to apply to make the user feel a force at the point PU */
  double f_perp = f_guide*(l_out/l_in);

  /* Calculate the force that the pantograph needs to generate
  beta is the angle between the force vector Fmech and the needle vector */
  double cos_beta = std::cos(phi)*std::cos(theta-alpha);
  double f_mech = f_perp*(l_in/l_out);

  return f_mech;
} 


}  // namespace pantograph_library
