"""The SDK lets you control an Orbita 3 dof actuator in Python.

It more details, with the SDK you can:

- **enable/disable the torque** of all 3 motors
- get Orbita's current position - you can either read the 3 **disks position** (in rads) or the plate **3D orientation** (as quaternion)
- set a new target position - you can either use disk position (in rads) or 3D orientation of the plate (as quaternion)
- get/set PID gains (same for all 3 motors)
- get each motor temperature (in degree celsius)

Communication with Orbita is rather fast (about ~1ms per command). So you can **control it at a few hundred Hz** (get its position and send a new target). Internal **PID controller runs at 1kHz**.

.. warning:: Communication performance may vary a lot depending on your serial driver! In particular, if you are using Windows, make sure to decrease the driver latency to 1ms. On Linux/Mac OS default driver should work fine.

Using the SDK is rather straightforward. You first need to instantiate an :class:`~orbita_sdk.OrbitaSDK`:

First, import the module:

>>> from orbita_sdk import OrbitaSDK

Instantiate a connection with the robot (only one connection should be open at the same time)

>>> orbita = OrbitaSDK(port='/dev/ttyACM0') # Make sure to use the correct serial port name!

Then you can directly send commands either to get a specific register value or send a new command.

For instance, to get the current 3D orientation:

>>> print(orbita.get_current_orientation())
[-1.69612761e-03  5.69884982e-04  1.50487807e-03  9.99997267e-01]

To send a command to enable the torque:

>>> orbita.enable_torque()

The motors of Orbita should now be hard and controlled.

"""

from typing import Any, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from .kinematics import OrbitaKinematicModel
from .register import OrbitaRegister
from .serial_io import OrbitaError, OrbitaSerialIO


class OrbitaSDK:
    """Handles communication with an Orbita actuator via a serial port.

    :param port: name of the serial port to use to communicate with Orbita
    :param baudrate: serial baudrate (only supports 500000 at the moment)
    :param timeout: serial timeout
    :param id: id of the Orbita you want to communicate with (default to 40)

    All calls to the hardware (read or write) are synchronous and blocking.

    """
    reduction = 52 / 24
    resolution = 4096

    _max_roll = 0.55
    _max_pitch = 0.55

    def __init__(self, port: str, baudrate: int = 500000, timeout=0.1, id: int = 40):
        """Open the serial communication."""
        self._io = OrbitaSerialIO(port=port, baudrate=baudrate, timeout=timeout)
        self._id = id
        self._kin = OrbitaKinematicModel()

        self._calibrate()

    def enable_torque(self):
        """Enable the torque on all 3 motors."""
        self._set_register(OrbitaRegister.GoalPosition, self._get_register(OrbitaRegister.PresentPosition))
        self._set_register(OrbitaRegister.TorqueEnable, (True, True, True))

    def disable_torque(self):
        """Disable the torque on all 3 motors."""
        self._set_register(OrbitaRegister.TorqueEnable, (False, False, False))

    def get_current_disk_position(self) -> Tuple[float, float, float]:
        """Get the current position of each disks.
        
        :return: Disks current position (in rads) in the following order (top, middle, bottom)
        """
        return self._pos_to_rad(self._get_register(OrbitaRegister.PresentPosition))

    def set_target_disk_position(self, target_position: Tuple[float, float, float]):
        """Set a new target disk position.

        :param target_position: Disks target positions (in rads), in the following order (top, middle, bottom).
        """
        print(self._rad_to_pos(target_position))
        self._set_register(OrbitaRegister.GoalPosition, self._rad_to_pos(target_position))

    def get_current_orientation(self) -> np.ndarray:
        """Get the current plate orientation.
        
        Please note, that an numerical method is used to compute the forward kinematics. This can lead to small inacurracies. Computation should take less than a ms on an average computer.

        :return: quaternion (qx, qy, qz, qw)
        """
        return self._kin.forward_kinematics(self.get_current_disk_position())

    def set_target_orientation(self, q: np.ndarray):
        """Set a new target orientation for the plate (as quaternion (qx, qy, qz, qw)).
        
        :param q: quaternion (qx, qy, qz, qw)

        The analytical inverse kinematics is used behind the scene to compute required disk position. Computation should take less than a ms on an average computer.
        """
        # roll, pitch, _ = Rotation.from_quat(q).as_euler('xyz')
        # if np.abs(roll) > self._max_roll or np.abs(pitch) > self._max_pitch:
        #     raise ValueError(f'Given quaternion out of range (should be with roll in [-{self._max_roll}, {self._max_roll}] and pitch in [-{self._max_pitch}, {self._max_pitch}] rads)')

        self.set_target_disk_position(self._kin.inverse_kinematics(q))

    def get_pid(self) -> Tuple[float, float, float]:
        """Get currently used PID gains.
        
        The PID is similar for all 3 motors for API simplicity.

        :return: (p, i, d) gains
        """
        return self._get_register(OrbitaRegister.PID)

    def set_pid(self, p: float, i: float, d: float):
        """Set new PID gains.

        :param p: p gain of the PID
        :param i: i gain of the PID
        :param d: d gain of the PID
        
        The PID is similar for all 3 motors for API simplicity.
        """
        self._set_register(OrbitaRegister.PID, (p, i, d))

    def get_current_temperature(self) -> Tuple[float, float, float]:
        """Get the current temperature of each disk motor.
        
        :return: Temperature (in °C) of each disk in the following order (top, middle, bottom).
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

    def _calibrate(self):
        def set_offset(raw_zero: int, raw_pos: int):
            """Set the correct offset depending on the hardware zero and the disk starting position."""
            possibilities = [
                raw_zero,
                raw_zero + self.resolution,
                raw_zero - self.resolution,
            ]
            distances = [abs(raw_pos - poss) for poss in possibilities]
            closest = np.argmin(distances)
            return possibilities[closest]

        pos = self._get_register(OrbitaRegister.AbsolutePosition)
        zero = self._get_register(OrbitaRegister.Zero)
        self._offset = np.array([set_offset(z, p) for z, p in zip(zero, pos)])
        print(pos)
        print(zero)
        print(self._offset)

        self._set_register(OrbitaRegister.Recalibrate, ())

    def _pos_to_rad(self, pos):
        pos = np.array(pos) - self._offset
        return tuple(2 * np.pi * pos / (self.reduction * self.resolution))

    def _rad_to_pos(self, rad):
        return tuple(
            int(p)
            for p in (np.array(rad) * self.reduction * self.resolution) / (2 * np.pi)
        ) + self._offset