# Copyright 2023, ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Thibault Poignonec (tpoignonec@unistra.fr)

import numpy as np


class PantographModel:
    # Static attributes
    __a1 = 0.100
    __a2 = 0.165
    __a5 = 0.085
    __l_needle = 0.2

    # Insertion point coords according to CAD
    PI_x = 0.0425
    PI_y = 0.16056
    PI_z = 0.09

    def __init__(self):
        self.__a3 = self.__a2
        self.__a4 = self.__a1

    def populate_all_angles(self, q):
        p = self.fk(q)
        angle_values = self.ik_full(p)
        return angle_values

    def fk(self, q):
        assert (q.size == 2)
        # Coordinates of P2 with respect to P1
        x2 = self.a1 * np.cos(q[0])
        y2 = self.a1 * np.sin(q[0])
        P2 = np.array([x2, y2])
        # Coordinates of P4 with respect to P1
        x4 = self.a4 * np.cos(q[1]) + self.a5
        y4 = self.a4 * np.sin(q[1])
        P4 = np.array([x4, y4])
        # FKM
        l24 = np.linalg.norm(P2 - P4)
        gamma = np.arccos(
            (self.a2**2 + l24**2 - self.a3**2)
            / (2 * self.a2 * l24)
        )
        delta = np.arctan2(y4 - y2, x4 - x2)
        theta_2 = gamma + delta

        x3 = x2 + self.a2 * np.cos(theta_2)
        y3 = y2 + self.a2 * np.sin(theta_2)

        return np.array([x3, y3])

    def ik(self, p):
        q_full = self.ik_full(p)
        return np.array([q_full[0], q_full[4]])

    def ik_full(self, p):
        assert (p.size == 2)

        P3 = p
        P5 = np.array([0 + self.a5, 0.0])
        a13 = np.linalg.norm(P3)
        a53 = np.linalg.norm(P3 - P5)

        alpha_1 = np.arccos(
            (self.a1**2 + a13**2 - self.a2**2)
            / (2 * self.a1 * a13)
        )
        beta_1 = np.arctan2(P3[1], P3[0])
        q1 = alpha_1 + beta_1

        alpha_5 = np.arccos(
            (self.a4**2 + a53**2 - self.a3**2)
            / (2 * self.a4 * a53)
        )
        beta_5 = np.arctan2(P3[1], P3[0] - P5[0])
        q5 = beta_5 - alpha_5

        # Passive joints
        # TODO: check the math here...
        beta_2 = np.arccos(((self.a2**2) + (a13**2)-(self.a1**2))/(2 * self.a2 * a13))
        beta_3 = np.arccos(((self.a3**2) + (a53**2)-(self.a4**2))/(2 * self.a3 * a53))
        beta_4 = np.pi - beta_3 - beta_5

        alpha_2 = np.pi - alpha_1 - beta_2
        alpha_3 = np.pi - alpha_5 - beta_1

        q2 = np.pi - alpha_2
        q3 = np.pi - beta_2 - alpha_3 - beta_3
        q4 = -(np.pi - beta_4)

        return np.array([q1, q2, q3, q4, q5])

    def jacobian(self, q):
        assert (q.size == 2)
        x2 = self.a1 * np.cos(q[0])
        y2 = self.a1 * np.sin(q[0])
        P2 = np.array([x2, y2])
        # Coordinates of P4 with respect to P1
        x4 = self.a4 * np.cos(q[1]) + self.a5
        y4 = self.a4 * np.sin(q[1])
        P4 = np.array([x4, y4])
        # Coordinates of P3 with respect to P1
        P3 = self.fk(q)
        # Compute theta 2 (see report)
        l24 = np.linalg.norm(P2 - P4)
        gamma = np.arccos(
            (self.a2**2 + l24**2 - self.a3**2)
            / (2 * self.a2 * l24)
        )
        delta = np.arctan2(y4 - y2, x4 - x2)
        theta2 = gamma + delta
        # Compute theta 4
        # Check math, that seems wrong...
        theta4 = np.arctan2(P3[1] - y4, P3[0] - x4)
        '''
        print('theta_2 = ', theta2 * 180 / np.pi, 'rad')
        print('vect 43 = ', P3 - P4, 'rad')
        print('theta_4 = ', theta4 * 180 / np.pi, 'rad')
        '''

        # Jacobian coefficients according to T.CESARE report
        S14S42 = np.sin(q[0] - theta4) / np.sin(theta4 - theta2)
        S24S42 = np.sin(q[1] - theta4) / np.sin(theta4 - theta2)

        J11 = -self.a1 * (np.sin(q[0]) + np.sin(theta2) * S14S42)
        J12 = self.a4 * np.sin(theta2) * S24S42
        J21 = self.a1 * (np.cos(q[0]) + np.cos(theta2) * S14S42)
        J22 = -self.a4 * np.cos(theta2) * S24S42

        return np.array([[J11, J12], [J21, J22]])

    def fk_system(self, q):
        # FKM of the complete system : calculates cartesian position
        # of interaction point PU according to joint space coordinates q
        PI = np.array([self.PI_x, self.PI_y, self.PI_z])

        # Get coords of P3 in base frame according to panto FKM
        P3_2D = self.fk(q)

        # Convert P3 coords in plane to coords in 3D
        P3 = np.append(P3_2D, 0)

        # Convert coords of PI in base frame to P3 frame
        P3I = PI - P3

        # Get azimuth (theta) and elevation (phi) angles (in radians) of vector P3PI
        theta = np.arctan2(P3I[1], P3I[0])
        phi = np.arctan2(P3I[2], np.sqrt(P3I[0]**2 + P3I[1]**2))
        Lin = np.sqrt(P3I[0]**2 + P3I[1]**2 + P3I[2]**2)

        # Calculate translation from PI to PU
        Lout = self.l_needle - Lin
        PIU = Lout * np.array([np.cos(phi)*np.cos(theta), np.cos(phi)*np.sin(theta), np.sin(phi)])

        # Convert coords of PU in PI frame to base frame
        PU = PI + PIU

        return PU

    def ik_system(self, PU):
        # Inverse kinematics model of the complete system:
        # computes the angles of all the joints according
        # to the cartesian position of interaction point PU
        PI = np.array([self.PI_x, self.PI_y, self.PI_z])

        # Transformation of PU coords in base frame to coords in PI frame
        PIU = PI - PU

        # Get azimuth (theta) and elevation (phi) angles (in radians) of vector P3PI
        theta = np.arctan2(PIU[1], PIU[0])
        phi = np.arctan2(PIU[2], np.sqrt(PIU[0]**2 + PIU[1]**2))
        Lout = np.sqrt(PIU[0]**2 + PIU[1]**2 + PIU[2]**2)

        # Get length of the needle segment betzeen PI and P3
        Lin = self.l_needle - Lout

        PI3 = Lin * np.array([np.cos(phi)*np.cos(theta), np.cos(phi)*np.sin(theta), np.sin(phi)])

        # Convert coords of PU in PI frame to base frame
        P3 = PI + PI3

        # Convert P3 coords in 3D to coords in plane
        P3_2D = np.array([P3[0], P3[1]])

        # Get angle of the pantograph joints using IKM
        jnt_pos = self.ik_full(self, P3_2D)

        # Return pantograph joint angles + needle orientation angles
        jnt_ext_pos = np.append(jnt_pos, [theta, phi])
        return jnt_ext_pos

    def get_panto_force(self, PU, f_guide, alpha):
        # Get all the robot joint angles
        jnt_ext_pos = self.ik_system(PU)
        theta = jnt_ext_pos[5]
        phi = jnt_ext_pos[6]

        # Get insertion point height
        H = self.PI_z

        # Calculate length of the needle segment between P3 and PI
        Lin = H / np.sin(phi)

        # Calculate the length of the needle segment between PI and PU
        Lout = self.l_needle - Lin

        # Calculate the perpendicular component of the force that the system
        # needs to apply to create the force desired at point PU
        f_perp = f_guide * (Lout/Lin)

        # Calculate the force that the pantograph needs to generate
        # beta is the angle between the force vector f_mech and the needle
        cos_beta = np.cos(phi) * np.cos(theta - alpha)
        f_mech = f_perp / np.sqrt(1 - cos_beta ** 2)

        return f_mech
    # Read-only properties

    @property
    def a1(self):
        return self.__a1

    @property
    def a2(self):
        return self.__a2

    @property
    def a3(self):
        return self.__a3

    @property
    def a4(self):
        return self.__a4

    @property
    def a5(self):
        return self.__a5

    @property
    def l_needle(self):
        return self.__l_needle
