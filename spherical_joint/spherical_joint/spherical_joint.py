import numpy as np
import quaternion
from numpy import linalg as LA
from math import sin, cos, pi, sqrt, acos, atan2


class Actuator:
    def __init__(self, Pc_z=[0, 0, 89.4], Cp_z=[0, 0, 64.227], R=39.162):
        self.Pc_z = Pc_z
        self.Cp_z = Cp_z
        self.R = R

    x0 = [1, 0, 0]
    y0 = [0, 1, 0]
    z0 = [0, 0, 1]
    x0_quat = quaternion.quaternion(0, 1, 0, 0)
    y0_quat = quaternion.quaternion(0, 0, 1, 0)
    z0_quat = quaternion.quaternion(0, 0, 0, 1)

    last_angles = [0, 2*pi/3, -2*pi/3]
    offset = [0, 0, 0]

    def get_new_frame_from_vector(self, vector, angle=0):
        beta = angle*pi/180

        # GOAL VECTOR (the desired Z axis)
        goal = vector
        goal_norm = []
        for i in goal:
            goal_norm.append(i/LA.norm(goal))  # Normalized vector of goal

        # VECTOR AND ANGLE OF ROTATION
        vec = np.cross(self.z0, goal_norm)

        vector_norm = []  # Normalized vector of rotation
        for i in vec:
            vector_norm.append(i/LA.norm(vec))

        alpha = acos(np.vdot(self.z0, goal_norm))  # Angle of rotation

        if alpha == 0:
            v = quaternion.quaternion(0.0, 0.0, 0.0, 1.0)

        else:  # Vector of rotation as a quaternion
            v = quaternion.quaternion(0.0, vector_norm[0], vector_norm[1],
                                      vector_norm[2])

        # QUATERNION OF ROTATION ###
        w1 = cos(alpha/2.0)
        x1 = sin(alpha/2.0)*v.x
        y1 = sin(alpha/2.0)*v.y
        z1 = sin(alpha/2.0)*v.z

        q1 = quaternion.quaternion(w1, x1, y1, z1)  # 1st rotation quaternion
        q1_inv = q1.inverse()

        z_prime = q1*self.z0_quat*q1_inv

        w2 = cos(beta/2.0)
        x2 = sin(beta/2.0)*z_prime.x
        y2 = sin(beta/2.0)*z_prime.y
        z2 = sin(beta/2.0)*z_prime.z

        # Quaternion of the rotation on new z axis
        q2 = quaternion.quaternion(w2, x2, y2, z2)
        q2_inv = q2.inverse()

        new_z = q2*z_prime*q2_inv  # Final Z
        new_x = q2*(q1*self.x0_quat*q1_inv)*q2_inv  # Final X
        new_y = q2*(q1*self.y0_quat*q1_inv)*q2_inv  # Final Y

        X = [new_x.x, new_x.y, new_x.z]
        Y = [new_y.x, new_y.y, new_y.z]
        Z = [new_z.x, new_z.y, new_z.z]

        return X, Y, Z

    def get_angles_from_vector(self, vector, angle=0):
        # Find q31_0 and Q11_0
        beta = angle
        goal = vector

        R = self.R
        Pc = self.Pc_z
        C = self.Cp_z

        X, Y, Z = self.get_new_frame_from_vector(goal)
        q31_0_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                  R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                  (R*Z[2] + C[2] - Pc[2])),
                  2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                          R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2]
                          - Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q31_0_[0] <= 180:
            q31_0 = q31_0_[0]
        else:
            q31_0 = q31_0_[1]
        num1 = Z[1]*cos(q31_0)+X[1]*sin(q31_0)
        den1 = Z[0]*cos(q31_0)+X[0]*sin(q31_0)
        q11_0 = atan2(num1, den1)

        # Find q32_0 and q12_0
        X, Y, Z = self.get_new_frame_from_vector(goal, 120)
        q32_0_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                  R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                  (R*Z[2] + C[2] - Pc[2])),
                  2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                          R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2]
                          - Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q32_0_[0] <= 180:
            q32_0 = q32_0_[0]
        else:
            q32_0 = q32_0_[1]
        num2 = (Z[1]*cos(q32_0)+X[1]*sin(q32_0))
        den2 = (Z[0]*cos(q32_0)+X[0]*sin(q32_0))
        q12_0 = atan2(num2, den2)

        # Find q33_0 and q13_0 ###
        X, Y, Z = self.get_new_frame_from_vector(goal, -120)
        q33_0_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                  R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                  (R*Z[2] + C[2] - Pc[2])),
                  2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                           R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2]
                           - Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q33_0_[0] <= 180:
            q33_0 = q33_0_[0]
        else:
            q33_0 = q33_0_[1]
        num3 = (Z[1]*cos(q33_0)+X[1]*sin(q33_0))
        den3 = (Z[0]*cos(q33_0)+X[0]*sin(q33_0))
        q13_0 = atan2(num3, den3)

        # Find q31 and q11
        X, Y, Z = self.get_new_frame_from_vector(goal, beta)
        q31_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                (R*Z[2] + C[2] - Pc[2])),
                2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                         R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] -
                         Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q31_[0] <= 180:
            q31 = q31_[0]
        else:
            q31 = q31_[1]
        num1 = Z[1]*cos(q31)+X[1]*sin(q31)
        den1 = Z[0]*cos(q31)+X[0]*sin(q31)
        q11 = atan2(num1, den1)

        # Find q32 and q12
        X, Y, Z = self.get_new_frame_from_vector(goal, beta+120)
        q32_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                (R*Z[2] + C[2] - Pc[2])),
                2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                         R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] -
                         Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q32_[0] <= 180:
            q32 = q32_[0]
        else:
            q32 = q32_[1]
        num2 = Z[1]*cos(q32)+X[1]*sin(q32)
        den2 = Z[0]*cos(q32)+X[0]*sin(q32)
        q12 = atan2(num2, den2)

        # Find q33 and q13
        X, Y, Z = self.get_new_frame_from_vector(goal, beta-120)
        q33_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                (R*Z[2] + C[2] - Pc[2])),
                2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                         R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] -
                         Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q33_[0] <= 180:
            q33 = q33_[0]
        else:
            q33 = q33_[1]
        num3 = (Z[1]*cos(q33)+X[1]*sin(q33))
        den3 = (Z[0]*cos(q33)+X[0]*sin(q33))
        q13 = atan2(num3, den3)

        if beta > 0:
            if q11 < q11_0:
                q11 = q11+2*pi
            if q12 < q12_0:
                q12 = q12+2*pi
            if q13 < q13_0:
                q13 = q13+2*pi

        if beta < 0:
            if q11 > q11_0:
                q11 = q11-2*pi
            if q12 > q12_0:
                q12 = q12-2*pi
            if q13 > q13_0:
                q13 = q13-2*pi

        q11 = q11*180/pi
        q12 = (q12*180/pi)-120
        q13 = (q13*180/pi)+120

        if abs(self.last_angles[0]-q11) > 360-abs(self.last_angles[0]-q11):
            self.offset[0] = self.offset[0] + \
                np.sign(self.last_angles[0]-q11)*360

        if abs(self.last_angles[1]-q12) > 360-abs(self.last_angles[1]-q12):

            self.offset[1] = self.offset[1] + \
                np.sign(self.last_angles[1]-q12)*360

        if abs(self.last_angles[2]-q13) > 360-abs(self.last_angles[2]-q13):
            self.offset[2] = self.offset[2] + \
                np.sign(self.last_angles[2]-q13)*360

        self.last_angles = [q11, q12, q13]

        q11 = q11+self.offset[0]
        q12 = q12+self.offset[1]
        q13 = q13+self.offset[2]

        return q11, q12, q13

    def get_new_frame_from_quaternion(self, Q):
        if isinstance(Q, tuple) or isinstance(Q, list):
            q1 = quaternion.quaternion(Q[0], Q[1], Q[2], Q[3])
        else:
            q1 = Q
        q1_inv = q1.inverse()

        new_z = q1*self.z0_quat*q1_inv  # Final Z
        new_x = q1*self.x0_quat*q1_inv  # Final X
        new_y = q1*self.y0_quat*q1_inv  # Final Y

        X = [new_x.x, new_x.y, new_x.z]
        Y = [new_y.x, new_y.y, new_y.z]
        Z = [new_z.x, new_z.y, new_z.z]

        return X, Y, Z

    def get_angles_from_quaternion(self, Q):
        R = self.R
        Pc = self.Pc_z
        C = self.Cp_z

        if isinstance(Q, tuple) or isinstance(Q, list):
            quat = quaternion.quaternion(Q[0], Q[1], Q[2], Q[3])
        else:  # If type of Q is quaternion
            quat = Q
        # Find q31 and q11 ###
        X, Y, Z = self.get_new_frame_from_quaternion(quat)
        q31_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                (R*Z[2] + C[2] - Pc[2])),
                2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                         R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] -
                         Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q31_[0] <= 180:
            q31 = q31_[0]
        else:
            q31 = q31_[1]
        num1 = Z[1]*cos(q31)+X[1]*sin(q31)
        den1 = Z[0]*cos(q31)+X[0]*sin(q31)
        q11 = atan2(num1, den1)

        # Find q32 and q12
        w_offset = cos(2*pi/6.0)
        x_offset = sin(2*pi/6.0)*self.z0_quat.x
        y_offset = sin(2*pi/6.0)*self.z0_quat.y
        z_offset = sin(2*pi/6.0)*self.z0_quat.z

        # 1st rotation quaternion
        q_offset = quaternion.quaternion(w_offset, x_offset,
                                         y_offset, z_offset)

        X, Y, Z = self.get_new_frame_from_quaternion(quat*q_offset)
        q32_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                (R*Z[2] + C[2] - Pc[2])),
                2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                         R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] -
                         Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q32_[0] <= 180:
            q32 = q32_[0]
        else:
            q32 = q32_[1]
        num2 = Z[1]*cos(q32)+X[1]*sin(q32)
        den2 = Z[0]*cos(q32)+X[0]*sin(q32)
        q12 = atan2(num2, den2)

        # Find q33 and q13
        w_offset = cos(-2*pi/6.0)
        x_offset = sin(-2*pi/6.0)*self.z0_quat.x
        y_offset = sin(-2*pi/6.0)*self.z0_quat.y
        z_offset = sin(-2*pi/6.0)*self.z0_quat.z

        # 1st rotation quaternion
        q_offset = quaternion.quaternion(w_offset, x_offset,
                                         y_offset, z_offset)

        X, Y, Z = self.get_new_frame_from_quaternion(quat*q_offset)
        q33_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                (R*Z[2] + C[2] - Pc[2])),
                2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                         R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] -
                         Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q33_[0] <= 180:
            q33 = q33_[0]
        else:
            q33 = q33_[1]
        num3 = (Z[1]*cos(q33)+X[1]*sin(q33))
        den3 = (Z[0]*cos(q33)+X[0]*sin(q33))
        q13 = atan2(num3, den3)

        last_angles = self.last_angles

        if (abs(q11-last_angles[0]) >= 2.96):
            if last_angles[0] > 0:
                q11 = q11+2*pi
            elif last_angles[0] < 0:
                q11 = q11-2*pi
        if (abs(q12-last_angles[1]) >= 2.96):
            if last_angles[1] > 0:
                q12 = q12+2*pi
            elif last_angles[1] < 0:
                q12 = q12-2*pi
        if (abs(q13-last_angles[2]) >= 2.96):
            if last_angles[2] > 0:
                q13 = q13+2*pi
            elif last_angles[2] < 0:
                q13 = q13-2*pi

        self.last_angles = [q11, q12, q13]

        return [q11*180/pi, (q12*180/pi)-120, (q13*180/pi)+120]

    def reset_last_angles(self):
        self.last_angles = [0, 2*pi/3, -2*pi/3]

    def reset_offset(self):
        self.offset = [0, 0, 0]
