"""
Notes
-----
This module is used to create an objet "Actuator" associated to the spherical
joint in order to control it.

In the end, the module can be used to calculate the angles in the disks needed
to make the platform follow a vector or a quaternion provided by another device
(IMU, trackers...)
"""


import numpy as np
from pyquaternion import Quaternion
from numpy import linalg as LA
from math import sin, cos, pi, sqrt, acos, atan2


class Actuator:
    """
    This actuator is composed of three disks, linked to three arms and a
    platform in the end. The goal is to orientate the platform, so the disks do
    a rotation following a circle called "proximal circle".
    Then, these disks make the arm rotate around the platform's center on a
    circle called "distal circle".

    Three parameters need to be set : The distal radius R and the 3D
    coordinates of the centers of the distal circle and the proximal circle.

    The mathematical explanation can be found in the spherical_symbolic.ipynb
    notebook
    """
    def __init__(self, Pc_z=[0, 0, 89.4],
                 Cp_z=[0, 0, 64.227], R=39.162,
                 R0=[[1, 0, 0], [0, 1, 0], [0, 0, 1]]):
        self.Pc_z = np.array(Pc_z)
        self.Cp_z = np.array(Cp_z)
        self.R = R

        self.x0 = np.array(R0[0])
        self.y0 = np.array(R0[1])
        self.z0 = np.array(R0[2])
        self.x0_quat = Quaternion(0, self.x0[0], self.x0[1], self.x0[2])
        self.y0_quat = Quaternion(0, self.y0[0], self.y0[1], self.y0[2])
        self.z0_quat = Quaternion(0, self.z0[0], self.z0[1], self.z0[2])

        self.last_angles = [0, 2*pi/3, -2*pi/3]
        self.offset = [0, 0, 0]

    def get_new_frame_from_vector(self, vector, angle=0):
        """
        Compute the coordinates of the vectors of a new frame whose Z axis is
        the chosen vector

        Parameters
        ----------
        vector : array_like
            Vector used to orientate the platform
        angle : float
            The desired angle of rotation of the platform on its Z axis
            in degrees

        Returns
        -------
        X : array_like
            New X vector of the platform's frame
        Y : array_like
            New Y vector of the platform's frame
        Z : array_like
            New Z vector of the platform's frame
        """

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
            v = Quaternion(0.0, 0.0, 0.0, 1.0)

        else:  # Vector of rotation as a quaternion
            v = Quaternion(0.0, vector_norm[0], vector_norm[1], vector_norm[2])

        # QUATERNION OF ROTATION ###
        w1 = cos(alpha/2.0)
        x1 = sin(alpha/2.0)*v.x
        y1 = sin(alpha/2.0)*v.y
        z1 = sin(alpha/2.0)*v.z

        q1 = Quaternion(w1, x1, y1, z1)  # 1st rotation quaternion
        q1_inv = q1.inverse

        z_prime = q1*self.z0_quat*q1_inv

        w2 = cos(beta/2.0)
        x2 = sin(beta/2.0)*z_prime.x
        y2 = sin(beta/2.0)*z_prime.y
        z2 = sin(beta/2.0)*z_prime.z

        # Quaternion of the rotation on new z axis
        q2 = Quaternion(w2, x2, y2, z2)
        q2_inv = q2.inverse

        new_z = q2*z_prime*q2_inv  # Final Z
        new_x = q2*(q1*self.x0_quat*q1_inv)*q2_inv  # Final X
        new_y = q2*(q1*self.y0_quat*q1_inv)*q2_inv  # Final Y

        X = [new_x.x, new_x.y, new_x.z]
        Y = [new_y.x, new_y.y, new_y.z]
        Z = [new_z.x, new_z.y, new_z.z]

        return X, Y, Z

    # FIXME: too complex
    def get_angles_from_vector(self, vector, angle=0):  # noqa: C901
        """
        Compute the angles of the disks needed to rotate the platform to the
        new frame, using the get_new_frame_from_vector function.

        The expression of q3 and q1 angles are found with the notebook
        spherical_symbolic.ipynb

        Parameters
        ----------
        vector : array_like
            Vector used to orientate the platform
        angle : float
            The desired angle of rotation of the platform on its Z axis
            in degrees

        Returns
        -------
        q11 : float
            angle of the top disk in degrees
        q12 : float
            angle of the middle disk in degrees
        q13 : float
            angle of the bottom disk in degrees
        """
        # First, get the initial postions q11_0, q12_0 and q13_0

        # Find q31_0 and q11_0
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

        if 0 <= q31_0_[0]*180/pi <= 180:
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

        if 0 <= q32_0_[0]*180/pi <= 180:
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

        if 0 <= q33_0_[0]*180/pi <= 180:
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

        if 0 <= q31_[0]*180/pi <= 180:
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

        if 0 <= q32_[0]*180/pi <= 180:
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

        if 0 <= q33_[0]*180/pi <= 180:
            q33 = q33_[0]
        else:
            q33 = q33_[1]
        num3 = (Z[1]*cos(q33)+X[1]*sin(q33))
        den3 = (Z[0]*cos(q33)+X[0]*sin(q33))
        q13 = atan2(num3, den3)

        # If there is a discontinuity, add or remove 2*pi radians dependin on
        # the sign of beta
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

        # If the difference between current position and 360° is low,
        # add or remove 360° to the offset applied on disks positions depending
        # on the sign of this difference
        if abs(self.last_angles[0]-q11) > 360.01-abs(self.last_angles[0]-q11):
            self.offset[0] = self.offset[0] + \
                np.sign(self.last_angles[0]-q11)*360

        if abs(self.last_angles[1]-q12) > 360.01-abs(self.last_angles[1]-q12):

            self.offset[1] = self.offset[1] + \
                np.sign(self.last_angles[1]-q12)*360

        if abs(self.last_angles[2]-q13) > 360.01-abs(self.last_angles[2]-q13):
            self.offset[2] = self.offset[2] + \
                np.sign(self.last_angles[2]-q13)*360

        self.last_angles = [q11, q12, q13]

        q11 = q11+self.offset[0]
        q12 = q12+self.offset[1]
        q13 = q13+self.offset[2]

        return q11, q12, q13

    def get_new_frame_from_quaternion(self, qw, qx, qy, qz):
        """
        Compute the coordinates of the vectors of a new frame got by a rotation
        represented by a quaternion

        Parameters
        ----------
        qw : float
            w parameter of the quaternion used to rotate the platform
        qx : float
            x parameter of the quaternion used to rotate the platform
        qy : float
            y parameter of the quaternion used to rotate the platform
        qz : float
            z parameter of the quaternion used to rotate the platform

        Returns
        -------
        X : array_like
            New X vector of the platform's frame
        Y : array_like
            New Y vector of the platform's frame
        Z : array_like
            New Z vector of the platform's frame
        """

        q1 = Quaternion(qw, qx, qy, qz)
        q1_inv = q1.inverse

        new_z = q1*self.z0_quat*q1_inv  # Final Z
        new_x = q1*self.x0_quat*q1_inv  # Final X
        new_y = q1*self.y0_quat*q1_inv  # Final Y

        X = [new_x.x, new_x.y, new_x.z]
        Y = [new_y.x, new_y.y, new_y.z]
        Z = [new_z.x, new_z.y, new_z.z]

        return X, Y, Z

    # FIXME: too complex
    def get_angles_from_quaternion(self, qw, qx, qy, qz):  # noqa: C901
        """
        Compute the angles of the disks needed to rotate the platform to the
        new frame, using the get_new_frame_from_vector function.

        The expression of q3 and q1 angles are found with the notebook
        spherical_symbolic.ipynb

        Parameters
        ----------
        qw : float
            w parameter of the quaternion used to rotate the platform
        qx : float
            x parameter of the quaternion used to rotate the platform
        qy : float
            y parameter of the quaternion used to rotate the platform
        qz : float
            z parameter of the quaternion used to rotate the platform

        Returns
        -------
        q11 : float
            angle of the top disk in degrees
        q12 : float
            angle of the middle disk in degrees
        q13 : float
            angle of the bottom disk in degrees
        """

        R = self.R
        Pc = self.Pc_z
        C = self.Cp_z

        quat = Quaternion(qw, qx, qy, qz)
        # Find q31 and q11 ###
        X, Y, Z = self.get_new_frame_from_quaternion(qw, qx, qy, qz)
        q31_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                (R*Z[2] + C[2] - Pc[2])),
                2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                         R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] -
                         Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q31_[0]*180/pi <= 180:
            q31 = q31_[0]
        else:
            q31 = q31_[1]
        num1 = Z[1]*cos(q31)+X[1]*sin(q31)
        den1 = Z[0]*cos(q31)+X[0]*sin(q31)
        q11 = atan2(num1, den1)

        # Find q32 and q12
        # Add an offset of +120°
        w_offset = cos(2*pi/6.0)
        x_offset = sin(2*pi/6.0)*self.z0_quat.x
        y_offset = sin(2*pi/6.0)*self.z0_quat.y
        z_offset = sin(2*pi/6.0)*self.z0_quat.z

        q_offset = Quaternion(w_offset, x_offset, y_offset, z_offset)
        Q = quat*q_offset
        X, Y, Z = self.get_new_frame_from_quaternion(Q.w, Q.x, Q.y, Q.z)
        q32_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                (R*Z[2] + C[2] - Pc[2])),
                2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                         R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] -
                         Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q32_[0]*180/pi <= 180:
            q32 = q32_[0]
        else:
            q32 = q32_[1]
        num2 = Z[1]*cos(q32)+X[1]*sin(q32)
        den2 = Z[0]*cos(q32)+X[0]*sin(q32)
        q12 = atan2(num2, den2)

        # Find q33 and q13
        # Add an offset of -120°
        w_offset = cos(-2*pi/6.0)
        x_offset = sin(-2*pi/6.0)*self.z0_quat.x
        y_offset = sin(-2*pi/6.0)*self.z0_quat.y
        z_offset = sin(-2*pi/6.0)*self.z0_quat.z

        q_offset = Quaternion(w_offset, x_offset, y_offset, z_offset)

        Q = quat*q_offset
        X, Y, Z = self.get_new_frame_from_quaternion(Q.w, Q.x, Q.y, Q.z)
        q33_ = [2*atan2((R*X[2] - sqrt(R**2*X[2]**2 +
                R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] - Pc[2]**2)),
                (R*Z[2] + C[2] - Pc[2])),
                2*atan2((R*X[2] + sqrt(R**2*X[2]**2 +
                         R**2*Z[2]**2 - C[2]**2 + 2*C[2]*Pc[2] -
                         Pc[2]**2)), (R*Z[2] + C[2] - Pc[2]))]

        if 0 <= q33_[0]*180/pi <= 180:
            q33 = q33_[0]
        else:
            q33 = q33_[1]
        num3 = (Z[1]*cos(q33)+X[1]*sin(q33))
        den3 = (Z[0]*cos(q33)+X[0]*sin(q33))
        q13 = atan2(num3, den3)

        last_angles = self.last_angles

        # If there are discontinuities, add or remove 2*pi radians depending on
        # The sign of the last angles
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
        """
        reset the last_angles values, used to know the previous angles of the
        disks
        """
        self.last_angles = [0, 2*pi/3, -2*pi/3]

    def reset_offset(self):
        """
        reset the offset values, used to know the number of full tours done by
        the disks
        """
        self.offset = [0, 0, 0]
