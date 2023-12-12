import numpy as np

class PantographModel:
    # Static attributs
    __a1 = 0.100
    __a2 = 0.165
    __a3 = self.__a2
    __a4 = self.__a1
    __a5 = 0.085

    def __init__(self):
        pass

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
        P4 = np.array([self.a4 * np.cos(q5) - self.a5, a4 * np.sin(q5)])
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
