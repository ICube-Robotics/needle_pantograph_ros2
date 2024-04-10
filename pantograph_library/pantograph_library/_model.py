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

    @staticmethod
    def populate_all_angles(self, q):
        p = self.fk(q)
        angle_values = self.ik(p)
        return angle_values

    @staticmethod
    def fk(self, q):
        # Coordinates of P2 with respect to P1
        x2 = self.a1 * np.cos(q[0])
        y2 = self.a1 * np.sin(q[0])
        P2 = np.array([x2, y2])
        # Coordinates of P4 with respect to P1
        x4 = self.a4 * np.cos(q[1]) - self.a5
        y4 = self.a4 * np.sin(q[1])
        P4 = np.array([x4, y4])
        # FKM
        P2Ph = (self.a2 * 2 - self.a3 * 2 + np.linalg.norm(P4 - P2) ** 2) / (2 * np.linalg.norm(P4 - P2))
        Ph = P2 + (P2Ph / np.linalg.norm(P2 - P4)) * (P4 - P2)
        P3Ph = np.sqrt(self.a2 * 2 - P2Ph * 2)
        # Coordinates of P3 with respect to P1
        x3 = Ph[0] + (P3Ph / np.linalg.norm(P2 - P4)) * (P4[1] - P2[1])
        y3 = Ph[1] - (P3Ph / np.linalg.norm(P2 - P4)) * (P4[0] - P2[0])
        return np.array([x3, y3])

    @staticmethod
    def ik(self, p):
        assert (p.size == 2)

        px = p[0]
        py = p[1]
        a13 = np.sqrt(px**2 + py**2)
        a53 = np.sqrt(((px + self.a5)**2) + py**2)

        alpha1 = np.arccos(((self.a1**2) + (a13**2) - (self.a2**2)) / (2 * self.a1 * a13))
        beta1 = np.arctan2(py, -px)
        theta1 = np.pi - alpha1 - beta1

        alpha5 = np.arctan2(py, px + self.a5)
        beta5 = np.arccos(((a53**2) + (np.a4**2) - (self.a3**2)) / (2 * a53 * self.a4))
        theta5 = alpha5 + beta5

        # Passive joints
        beta2 = np.arccos(((self.a2**2) + (a13**2) - (self.a1**2)) / (2 * self.a2 * a13))
        beta3 = np.arccos(((self.a3**2) + (a53**2) - (self.a4**2)) / (2 * self.a3 * a53))
        beta4 = np.pi - beta3 - beta5
        alpha2 = np.pi - alpha1 - beta2
        alpha3 = np.pi - alpha5 - beta1

        theta2 = np.pi - alpha2
        theta3 = np.pi - beta2 - alpha3 - beta3
        theta4 = -(np.pi - beta4)

        return np.array([theta1, theta2, theta3, theta4, theta5])

    @staticmethod
    def jacobian(self, q):
        q1 = q[0]
        q5 = q[1]
        # position of P2 and P4
        P2 = np.array([self.a1 * np.cos(q1), self.a1 * np.sin(q1)])
        P4 = np.array([self.a4 * np.cos(q5) - self.a5, self.a4 * np.sin(q5)])
        x2, y2 = P2
        x4, y4 = P4

        d = np.linalg.norm(P2 - P4)
        b = ((self.a2 * 2) - (self.a3 * 2) + (d ** 2)) / (2 * d)
        h = np.sqrt((self.a2 * 2) - (b * 2))

        d1x2 = (-self.a1) * np.sin(q1)
        d1y2 = self.a1 * np.cos(q1)
        d5x4 = (-self.a4) * np.sin(q5)
        d5y4 = self.a4 * np.cos(q5)
        dx2 = np.array([d1x2, 0])
        dy2 = np.array([d1y2, 0])
        dx4 = np.array([0, d5x4])
        dy4 = np.array([0, d5y4])

        dd = (((x4 - x2) * (dx4 - dx2)) + ((y4 - y2) * (dy4 - dy2))) / d
        db = dd - ((dd * (self.a2 * 2 - self.a3 * 2 + d * 2)) / (2 * d * 2))
        dh = -(b * db / h)
        dyh = dy2 + ((y4 - y2) / d ** 2) * (db * d - dd * b) + (dy4 - dy2) * (b / d)
        dxh = dx2 + ((x4 - x2) / d ** 2) * (db * d - dd * b) + (dx4 - dx2) * (b / d)

        dy3 = dyh - ((x4 - x2) / d ** 2) * (dh * d - dd * h) - (dx4 - dx2) * (h / d)
        dx3 = dxh + ((y4 - y2) / d ** 2) * (dh * d - dd * h) + (dy4 - dy2) * (h / d)
        J = np.vstack([dx3, dy3])

        return J

    # Models of the complete system (pantograph + needle)
    @staticmethod
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

    @staticmethod
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
        jnt_pos = self.ik(self, P3_2D)

        # Return pantograph joint angles + needle orientation angles
        jnt_ext_pos = np.append(jnt_pos, [theta, phi])
        return jnt_ext_pos

    @staticmethod
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
