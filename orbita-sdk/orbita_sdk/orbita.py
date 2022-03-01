"""Orbita Control SDK.

The SDK lets you control an Orbita 3Dof actuator.

It allows you to:
- enable/disable the torque
- get its current position - you can either read the 3 disks position (in rads) or the plate 3D orientation (as quaternion)
- set a new target position - you can either use disk position (in rads) or 3D orientation of the plate (as quaternion)
- get/set PID gains use for the control of all 3 motors
- get each motor temperature (in degree celsius)

"""

from typing import Any, Tuple

import numpy as np

from .kinematics import OrbitaKinematicModel
from .register import OrbitaRegister
from .serial_io import OrbitaError, OrbitaSerialIO


class OrbitaSDK:
    reduction = 52 / 24
    resolution = 4096

    def __init__(self, port: str, baudrate: int = 500000, timeout=0.1, id: int = 40) -> None:
        """Open the serial communication."""
        self._io = OrbitaSerialIO(port=port, baudrate=baudrate, timeout=timeout)
        self._id = id
        self._kin = OrbitaKinematicModel()

    def enable_torque(self) -> None:
        """Enable the torque on all 3 motors."""
        self._set_register(OrbitaRegister.GoalPosition, self._get_register(OrbitaRegister.PresentPosition))
        self._set_register(OrbitaRegister.TorqueEnable, (True, True, True))

    def disable_torque(self) -> None:
        """Disable the torque on all 3 motors."""
        self._set_register(OrbitaRegister.TorqueEnable, (False, False, False))

    def get_current_disk_position(self) -> Tuple[float, float, float]:
        """Get the current position (in rads.) of each disks.
        
        The returned value are given in the following order (top disk, middle, bottom).
        """
        return self._pos_to_rad(self._get_register(OrbitaRegister.PresentPosition))

    def set_target_disk_position(self, target_position: Tuple[float, float, float]):
        """Set a new target disk position (in rads).
        
        The given positions must be in the following order (top disk, middle, bottom).
        """
        self._set_register(OrbitaRegister.GoalPosition, self._rad_to_pos(target_position))

    def get_current_orientation(self) -> np.ndarray:
        """Get the current plate orientation (as quaternion (qx, qy, qz, qw)).
        
        Please note, that an numerical method is used to compute the forward kinematics. This can lead to small inacurracies. Computation should take less than a ms on an average computer.
        """
        return self._kin.forward_kinematics(self.get_current_disk_position())

    def set_target_orientation(self, q: np.ndarray):
        """Set a new target orientation for the plate (as quaternion (qx, qy, qz, qw)).
        
        The analytical inverse kinematics is used behind the scene to compute required disk position. Computation should take less than a ms on an average computer.
        """
        self.set_target_disk_position(self._kin.inverse_kinematics(q))

    def get_pid(self) -> Tuple[float, float, float]:
        """Get currently used PID gains.
        
        The PID is similar for all 3 motors for API simplicity.
        """
        return self._get_register(OrbitaRegister.PID)

    def set_pid(self, p: float, i: float, d: float):
        """Set new PID gains.
        
        The PID is similar for all 3 motors for API simplicity.
        """
        self._set_register(OrbitaRegister.PID, (p, i, d))

    def get_current_temperature(self) -> Tuple[float, float, float]:
        """Get the current temperature (in degree celsius) of each disk motor.
        
        The returned value are given in the following order (top disk, middle, bottom).
        """        
        return self._get_register(OrbitaRegister.Temperature)

    def _get_register(self, reg: OrbitaRegister) -> Tuple[Any, Any, Any]:
        err, val = self._io.read(self._id, reg)
        self._check_error(err)
        return tuple(val)

    def _set_register(self, reg: OrbitaRegister, val: Tuple[Any, Any, Any]):
        err = self._io.write(self._id, reg, val)
        self._check_error(err)

    def _check_error(self, errors):
        # TODO
        pass

    def _pos_to_rad(self, pos):
        return tuple(2 * np.pi * np.array(pos) / (self.reduction * self.resolution))

    def _rad_to_pos(self, rad):
        return tuple(
            int(p)
            for p in (np.array(rad) * self.reduction * self.resolution) / (2 * np.pi)
        )