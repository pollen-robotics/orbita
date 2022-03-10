"""Orbita kinematic theoretical model."""
import pickle
from pathlib import Path
from typing import Tuple

import numpy as np
from numpy import linalg as LA

from pyquaternion import Quaternion

from scipy.spatial.transform import Rotation as R


def rot(axis, deg):
    """Compute 3D rotation matrix given euler rotation."""
    return R.from_euler(axis, np.deg2rad(deg)).as_matrix()


class OrbitaKinematicModel(object):
    """
    Orbita theoretical kinematic model.

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

    def __init__(self,
                 Pc_z: Tuple[float, float, float] = (0, 0, 89.4),
                 Cp_z: Tuple[float, float, float] = (0, 0, 64.227),
                 R: float = 39.162,
                 R0: np.ndarray = np.eye(3)):
        """Create a new actuator with the given disks configuration."""
        self.Pc_z = np.array(Pc_z)
        self.Cp_z = np.array(Cp_z)
        self.R = R
        self.R0 = np.array(R0)
        self.x0, self.y0, self.z0 = self.R0

        self.x0_quat = Quaternion(0, self.x0[0], self.x0[1], self.x0[2])
        self.y0_quat = Quaternion(0, self.y0[0], self.y0[1], self.y0[2])
        self.z0_quat = Quaternion(0, self.z0[0], self.z0[1], self.z0[2])

        self.last_angles = np.array([0, 2 * np.pi / 3, -2 * np.pi / 3])
        self.offset = np.array([0, 0, 0])

        import orbita_sdk
        path_model = Path(orbita_sdk.__file__).parent / 'mlpreg.obj'

        with open(path_model, 'rb') as f:
            self.model = pickle.load(f)

    def inverse_kinematics(self, q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        """Compute analytical inverse kinematics.
        
        :param q: quaternion (qx, qy, qz, qw)
        :return: disk positions (in rads) in the following order (top, middle, bottom)."""
        return np.deg2rad(self.get_angles_from_quaternion(q[3], q[0], q[1], q[2]))

    def forward_kinematics(self, disks: Tuple[float, float, float]) -> Tuple[float, float, float, float]:
        """Use KNN regression to compute an approximate forward kinematics.
        
        :param disks: disks position (in rads.) in the following order (top, middle, bottom)
        :return: quaternion (qx, qy, qz, qw)
        """
        rpy = self.model.predict(np.array(disks).reshape(1, -1))
        M1 = R.from_euler('XYZ', rpy).as_matrix()
        M = np.dot(M1, self.R0)
        q = R.from_matrix(M).as_quat()
        return q[0]

    def get_new_frame_from_quaternion(self, qw: float, qx: float, qy: float, qz: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute the coordinates of the vectors of a new frame got by a rotation represented by a quaternion.

        :param qw: w parameter of the quaternion used to rotate the platform
        :param qx: x parameter of the quaternion used to rotate the platform
        :param qy: y parameter of the quaternion used to rotate the platform
        :param qz: z parameter of the quaternion used to rotate the platform

        :return: (X, Y, Z) representing the new vectors of the platform's frame
        """
        q1 = Quaternion(qw, qx, qy, qz)
        q1_inv = q1.inverse

        new_z = q1 * self.z0_quat * q1_inv  # Final Z
        new_x = q1 * self.x0_quat * q1_inv  # Final X
        new_y = q1 * self.y0_quat * q1_inv  # Final Y

        X = np.array([new_x.x, new_x.y, new_x.z])
        Y = np.array([new_y.x, new_y.y, new_y.z])
        Z = np.array([new_z.x, new_z.y, new_z.z])

        return X, Y, Z

    # FIXME: too complex
    def get_angles_from_quaternion(self, qw: float, qx: float, qy: float, qz: float) -> Tuple[float, float, float]:  # noqa: C901
        """Compute the angles of the disks needed to rotate the platform to the new frame, using the get_new_frame_from_vector function.

        :param qw: w parameter of the quaternion used to rotate the platform
        :param qx: x parameter of the quaternion used to rotate the platform
        :param qy: y parameter of the quaternion used to rotate the platform
        :param qz: z parameter of the quaternion used to rotate the platform

        :return: disk position (in deg.) in following order (top, middle, bottom)
        """
        def get_frame(q):
            return self.get_new_frame_from_quaternion(q.w, q.x, q.y, q.z)

        quat = Quaternion(qw, qx, qy, qz)
        q31, q11 = self._eq(*get_frame(quat))

        # Find q32 and q12
        # Add an offset of +120°
        w_offset = np.cos(2 * np.pi / 6.0)
        x_offset = np.sin(2 * np.pi / 6.0) * self.z0_quat.x
        y_offset = np.sin(2 * np.pi / 6.0) * self.z0_quat.y
        z_offset = np.sin(2 * np.pi / 6.0) * self.z0_quat.z
        q_offset = Quaternion(w_offset, x_offset, y_offset, z_offset)
        Q = quat * q_offset
        q32, q12 = self._eq(*get_frame(Q))

        # Find q33 and q13
        # Add an offset of -120°
        w_offset = np.cos(-2 * np.pi / 6.0)
        x_offset = np.sin(-2 * np.pi / 6.0) * self.z0_quat.x
        y_offset = np.sin(-2 * np.pi / 6.0) * self.z0_quat.y
        z_offset = np.sin(-2 * np.pi / 6.0) * self.z0_quat.z
        q_offset = Quaternion(w_offset, x_offset, y_offset, z_offset)
        Q = quat * q_offset
        q33, q13 = self._eq(*get_frame(Q))

        last_angles = self.last_angles

        # If there are discontinuities, add or remove 2*pi radians depending on
        # The sign of the last angles
        if (abs(q11 - last_angles[0]) >= 2.96):
            if last_angles[0] > 0:
                q11 += 2 * np.pi
            elif last_angles[0] < 0:
                q11 -= 2 * np.pi
        if (abs(q12 - last_angles[1]) >= 2.96):
            if last_angles[1] > 0:
                q12 += 2 * np.pi
            elif last_angles[1] < 0:
                q12 -= 2 * np.pi
        if (abs(q13 - last_angles[2]) >= 2.96):
            if last_angles[2] > 0:
                q13 += 2 * np.pi
            elif last_angles[2] < 0:
                q13 -= 2 * np.pi

        self.last_angles = np.array([q11, q12, q13])

        return (
            np.rad2deg(q11),
            np.rad2deg(q12) - 120,
            np.rad2deg(q13) + 120,
        )

    def find_quaternion_transform(self, vect_origin: np.ndarray, vect_target: np.ndarray) -> Quaternion:
        """Find the quaternion to transform the vector origin to the target one.
        
        :param vect_origin: 3D origin vector
        :param vect_target: 3D target vector
        :return: quaternion rotation to go from origin to target

        """
        vo = np.array(vect_origin)
        if np.any(vo):
            vo = vo / LA.norm(vo)

        vt = np.array(vect_target)
        if np.any(vt):
            vt = vt / LA.norm(vt)

        V = np.cross(vo, vt)
        if np.any(V):
            V = V / LA.norm(V)

        alpha = np.arccos(np.dot(vo, vt))
        if np.isnan(alpha) or alpha < 1e-6:
            return Quaternion(1, 0, 0, 0)

        return Quaternion(axis=V, radians=alpha)

    def _eq(self, X, Y, Z):
        R = self.R
        Pc = self.Pc_z
        C = self.Cp_z

        d1 = (
            R**2 * X[2]**2 +
            R**2 * Z[2]**2 -
            C[2]**2 + 2 * C[2] * Pc[2] - Pc[2]**2
        )
        if d1 < 0:
            raise ValueError('math domain error')

        d1 = np.sqrt(d1)

        x11 = R * X[2] - d1
        x12 = R * X[2] + d1
        x2 = R * Z[2] + C[2] - Pc[2]

        sol1 = 2 * np.arctan2(x11, x2)
        sol2 = 2 * np.arctan2(x12, x2)

        if 0 <= np.rad2deg(sol1) <= 180:
            q3 = sol1
        else:
            q3 = sol2

        q1 = np.arctan2(
            Z[1] * np.cos(q3) + X[1] * np.sin(q3),
            Z[0] * np.cos(q3) + X[0] * np.sin(q3),
        )
        return q3, q1
