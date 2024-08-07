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
  Eigen::Vector2d P1, P2, P3, P4, P5;

  P1 << -l_a5_ / 2, 0;  // Coords of P1 in base frame
  P5 << l_a5_ / 2, 0;  // Coords of P2 in base frame

  // Coords of P2 in base frame
  P2[0] = P1[0] + l_a1_ * std::cos(q[0]);
  P2[1] = P1[1] + l_a1_ * std::sin(q[0]);

  // Coords of P4 in base frame
  P4[0] = P5[0] + l_a4_ * std::cos(q[1]);
  P4[1] = P5[1] + l_a4_ * std::sin(q[1]);

  // Calculate the angles of the different joints (relative to base frame)
  double a24 = (P2 - P4).norm();

  // Left arm
  double gamma2 =
    std::acos(
    (std::pow(a24, 2) + std::pow(l_a2_, 2) - std::pow(l_a3_, 2)) /
    (2 * a24 * l_a2_));

  double delta = std::atan2(P4[1] - P2[1], P4[0] - P2[0]);

  double theta2 = gamma2 + delta;

  // Right arm
  // double gamma4 =
  //   std::acos(
  //   (std::pow(a24, 2) + std::pow(l_a3_, 2) - std::pow(l_a2_, 2)) /
  //   (2 * a24 * l_a3_));

  // double theta4 = PI_CST - gamma4 - delta;

  // Coordinates of P3 according to left kinematic chain
  P3[0] = P2[0] + l_a2_ * std::cos(theta2);
  P3[1] = P2[1] + l_a2_ * std::sin(theta2);

  return P3;
}

Eigen::Vector<double, 5>
PantographModel::ik(Eigen::Vector<double, 2> P3)
{
  /* Inverse kinematics model of the pantograph,
   the angles thetai are used to simulate the position
  of the different robot segments in RViz */
  Eigen::Vector<double, 5> jnt_pos;
  Eigen::Vector2d P1, P5, P13, P53, x0;

  P1 << -l_a5_ / 2, 0;  // Coords of P1 in base frame
  P5 << l_a5_ / 2, 0;   // Coords of P5 in base frame

  // Calculate the length of the segment P1P3
  P13 = P3 - P1;
  double a13 = P13.norm();

  // Calculate the length of the segment P5P3
  P53 = P3 - P5;
  double a53 = P53.norm();

  // X axis for dot product
  x0 << 1, 0;

  // Triangle P1 P2 P3
  double alpha1 =
    std::acos(
    (std::pow(l_a1_, 2) - std::pow(l_a2_, 2) + std::pow(a13, 2)) /
    (2 * l_a1_ * a13));
  double alpha2 = std::acos(
    (std::pow(l_a1_, 2) + std::pow(l_a2_, 2) - std::pow(a13, 2)) /
    (2 * l_a1_ * l_a2_));
  double alpha3 = PI_CST - alpha2 - alpha1;

  // Triangle P5 P4 P3
  double alpha5 = std::acos(
    (std::pow(l_a4_, 2) - std::pow(l_a3_, 2) + std::pow(a53, 2)) /
    (2 * a53 * l_a4_));
  double alpha4 = std::acos(
    (std::pow(l_a4_, 2) + std::pow(l_a3_, 2) - std::pow(a53, 2)) /
    (2 * l_a3_ * l_a4_));
  double alpha3bis = PI_CST - alpha4 - alpha5;

  // Triangle P1 P5 P3
  double beta1 = std::atan2(P3[1], std::abs(P13.dot(x0)));
  double beta5 = std::atan2(P3[1], std::abs(P53.dot(x0)));
  double beta3 = PI_CST - beta1 - beta5;

  // // Calculate angle between P3 frame and tool frame
  // // in order to keep tool frame parallel to base frame
  // double delta3 = alpha3 + alpha3bis + beta3;

  /*Return the angles of all the joints for Rviz render
  angles are defined according to parent frame
  and are positive in trigonometric direction */

  jnt_pos[0] = alpha1 + beta1;  // = theta1 = q[0]
  jnt_pos[1] = -(PI_CST - alpha2);   // = theta2
  jnt_pos[2] = -jnt_pos[0] - jnt_pos[1];  // -(PI_CST - delta3);
  jnt_pos[3] = (PI_CST - alpha4);  // = theta4
  jnt_pos[4] = PI_CST - beta5 - alpha4;  // = theta5 = q[1]

  return jnt_pos;
}

Eigen::Matrix<double, 2, 2>
PantographModel::jacobian(Eigen::Vector<double, 2> q)
{
  Eigen::Vector2d P1, P2, P3, P4, P5;
  P3 = fk(q);
  P1 << -l_a5_ / 2, 0;
  P5 << l_a5_ / 2, 0;
  // Coords of P2 in base frame
  P2[0] = P1[0] + l_a1_ * std::cos(q[0]);
  P2[1] = P1[1] + l_a1_ * std::sin(q[0]);
  // Coords of P4 in base frame
  P4[0] = P5[0] + l_a4_ * std::cos(q[1]);
  P4[1] = P5[1] + l_a4_ * std::sin(q[1]);

  // Intermediate variables
  double a24 = (P2 - P4).norm();
  double gamma2 =
    std::acos(
    (std::pow(l_a2_, 2) + std::pow(a24, 2) - std::pow(l_a3_, 2)) /
    (2 * a24 * l_a2_));
  double delta = std::atan2(P4[1] - P2[1], P4[0] - P2[0]);

  double theta2 = gamma2 + delta;

  // Check math for theta4 calculation
  // double theta4 = std::atan2(P3[1] - P4[1], P3[0] - P4[0]);
  double gamma4 = PI_CST - std::acos(
    (std::pow(a24, 2) + std::pow(l_a3_, 2) - std::pow(l_a2_, 2)) /
    2 * a24 * l_a3_);

  double theta4 = PI_CST - gamma4 + delta;

  double S14S42 = sin(q(0) - theta4) / sin(theta4 - theta2);
  double S24S42 = sin(q(1) - theta4) / sin(theta4 - theta2);

  // Jacobian coefficients according to T.CESARE report
  double J11 = -l_a1_ * (sin(q(0)) + sin(theta2) * S14S42);
  double J12 = l_a4_ * sin(theta2) * S24S42;
  double J21 = l_a1_ * (cos(q(0)) + cos(theta2) * S14S42);
  double J22 = -l_a4_ * cos(theta2) * S24S42;

  Eigen::Matrix2d J;

  J << J11, J12,
    J21, J22;

  return J;
}

Eigen::Vector<double, 3>
PantographModel::fk_system(Eigen::Vector<double, 2> q)
{
  // FKM of the complete system : calculates cartesian position of interaction point PU
  // according to joint space coordinates q
  Eigen::Vector3d PI, P3, P3I, PIU, PU;

  // Coordinates of insertion point PI in base frame according to CAD
  PI[0] = PI_x;  // = 0;
  PI[1] = PI_y;  // = 0.16056;
  PI[2] = PI_z;  // = 0.09;

  // Get coords of P3 in base frame according to FKM
  Eigen::Vector2d P3_2D;
  P3_2D = fk(q);

  // Convert P3 in the plane to coords in 3D
  P3 << P3_2D, 0;

  // Convert coords of PI in base frame to P3 frame
  P3I = PI - P3;

  // Get angles (in radians) of vector P3PI according to P3 frame:
  // theta is the angle of rotation around z
  // phi is the angle of rotation around y
  double theta = std::atan2(P3I[1], P3I[0]);
  double phi = std::atan2(P3I[2], std::sqrt(std::pow(P3I[0], 2) + std::pow(P3I[1], 2)));
  // double Lin = std::sqrt(std::pow(P3I[0], 2) + std::pow(P3I[1], 2) + std::pow(P3I[2], 2));
  double Lin = PI[2] / std::sin(phi);

  // Calculate translation from PI to PU
  double Lout = l_needle_ - Lin;
  //  Coords of PU in PI frame
  PIU[0] = Lout * std::cos(phi) * std::cos(theta);
  PIU[1] = Lout * std::cos(phi) * std::sin(theta);
  PIU[2] = Lout * std::sin(phi);
  // PIU[2] = -Lout * std::sin(phi);

  // Convert coords of PU in PI frame to coords in base frame
  PU = PI + PIU;

  return PU;
}
Eigen::Vector<double, 8>
PantographModel::ik_system(Eigen::Vector<double, 3> PU)
{
  /* Inverse kinematics model of the complete system:
  computes the angles of all the joints according
  to the cartesian position of interaction point PU in base frame*/

  Eigen::Vector3d PI, P3, PI3, PIU;

  // Coords of insertion point PI in base frame according to CAD
  PI[0] = PI_x;
  PI[1] = PI_y;
  PI[2] = PI_z;

  // Transformation of PU coords in base frame to coords in PI frame
  PIU = PU - PI;

  // Get angles (in radians) of vector PIPU in PI frame
  // theta is the angle of rotation around z
  // phi is the angle of rotation around y
  double theta = std::atan2(PIU[1], PIU[0]);
  double phi = std::atan2(PIU[2], std::sqrt(std::pow(PIU[0], 2) + std::pow(PIU[1], 2)));
  double Lout = std::sqrt(std::pow(PIU[0], 2) + std::pow(PIU[1], 2) + std::pow(PIU[2], 2));

  // Get length of the needle segment between PI and P3
  double Lin = PI[2] / std::sin(phi);

  // convert phi for the correct definition of the angle in RViz
  phi = (PI_CST / 2) - phi;

  // Transformation from I to P3 in I frame
  PI3[0] = Lin * std::cos(phi) * std::cos(theta);
  PI3[1] = Lin * std::cos(phi) * std::sin(theta);
  PI3[2] = -Lin * std::sin(phi);

  // Transformation of P3 coords on I frame to coords in base frame
  P3 = PI + PI3;

  // Convert P3 coords in 3D to coords in plane
  Eigen::Vector<double, 2> P3_2D;
  P3_2D[0] = P3[0];
  P3_2D[1] = P3[1];

  // Get angles of the pantograph joints using IKM
  Eigen::Vector<double, 5> jnt_pos;
  jnt_pos = ik(P3_2D);

  // Return pantograph joints angles + needle orientation angles

  Eigen::Vector<double, 8> jnt_ext_pos;
  jnt_ext_pos << jnt_pos, theta, phi, Lout;

  return jnt_ext_pos;
}

// Calculate joint positions of the complete system for visualization
Eigen::Vector<double, 8>
PantographModel::populate_all_joint_positions_full_system(Eigen::Vector<double, 2> q)
{
  auto p = fk_system(q);
  return ik_system(p);
}


double
PantographModel::get_panto_force(Eigen::Vector<double, 3> PU, double f_guide, double alpha)
{
  /* Calculate force applied by the pantograph (f_mech)to create the force
  felt by the user with:
    - alpha : the angle determined by the direction
          of the force applied by the user
    - f_guide : the desired guiding force to be felt by the user*/

  // Get all the robot joint angles IKM
  Eigen::Vector<double, 8> jnt_ext_pos;
  jnt_ext_pos = ik_system(PU);
  double theta = jnt_ext_pos[5];  // Azimuth angle
  double phi = jnt_ext_pos[6];  // Elevation angle

  // Check for phi angle definition
  phi = (PI_CST / 2) - phi;

  // get insertion point height
  double H = PI_z;  // = 0.09 based on insertion point coords

  // Calculate length of the needle segment between P3 and PI
  double l_in = H / std::sin(phi);

  // Calculate length of the needle segment between PI and PU
  double l_out = l_needle_ - l_in;  // As a first approximation, L_out is constant

  /*Calculate the perpendicular component of the force that the system
  needs to apply to make the user feel a force at the point PU */
  double f_perp = f_guide * (l_out / l_in);

  /* Calculate the force that the pantograph needs to generate
  beta is the angle between the force vector Fmech and the needle vector */
  double cos_beta = std::cos(phi) * std::cos(theta - alpha);
  double f_mech = f_perp / std::sqrt(1 - std::pow(cos_beta, 2));

  // Calculate the folt felt by the user at point PU
  // double f_user = f_perp * (l_in / l_out);

  return f_mech;
}

Eigen::Vector3d
PantographModel::dk_system(Eigen::Vector<double, 2> q, Eigen::Vector<double, 2> dq)
{
  Eigen::Vector3d P3, PU, PI, PIU;

  // Get point P3 coords in 3D
  P3 << fk(q), 0;

  // Get point PU coords
  PU = fk_system(q);

  // Get theta, phi and beta1 angles
  PI << PI_x, PI_y, PI_z;
  PIU = PU - PI;

  double theta = std::atan2(PIU[1], PIU[0]);
  double phi = std::atan2(PIU[2], std::sqrt(std::pow(PIU[0], 2) + std::pow(PIU[1], 2)));
  double beta1 = std::atan2(P3[1], P3[0]);

  // Compute angle matrix M
  Eigen::Matrix3d M;
  M << 0, (beta1 + theta), 0,
    -(beta1 + theta), 0, phi * cos(beta1),
    0, -phi * cos(beta1), 0;

  // Get jacobian of the pantograph
  Eigen::MatrixX2d J1 = jacobian(q);
  // Resize jacobian adding a row of zeros
  J1.conservativeResize(J1.rows() + 1, J1.cols());
  J1.row(J1.rows() - 1) = Eigen::RowVector2d::Zero(2);

  // Compute velocity of point PU
  Eigen::Vector3d dPU;
  dPU = (J1 * dq) + M * (P3 - PU);

  return dPU;
}


}  // namespace pantograph_library
