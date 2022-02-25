"""Orbita Serial Communication.

This module implements the low-level communicaiton with Orbita acutators.
Most users should not need to access this level.

The communication with the Orbita Actuator is done using serial communication.
The message and protocol is close the Dynamixel v1 protocol.

You send an instruction message (ping, read, write) and you get a status packet as answer.
Each message is detailed in the documentation of its associated method.
The header, crc, and global message structure are respected such an Orbita can be connected to a Dynamixel buses (RS485) seamlessly.

"""
from typing import Any, List, Tuple

import logging
import numpy as np
import serial
import struct

from enum import Enum
from threading import Lock

from .register import OrbitaRegister


class Instruction(Enum):
    PING = 0x01
    READ_DATA = 0x02
    WRITE_DATA = 0x03


class OrbitaError(Enum):
    InputVoltage = 0
    AngleLimit = 1
    Overheating = 2
    Range = 3
    Checksum = 4
    Overload = 5
    Instruction = 6

    @classmethod
    def from_error_code(cls, err: np.uint8):
        byte = bin(err)[2:]
        byte = '0' * (8 - len(byte)) + byte
        return [
            cls(i) 
            for i, bit in enumerate(byte) 
            if bit == '1'
        ]


class OrbitaSerialIO:
    HEADER_SIZE = 4

    def __init__(self, port: str, baudrate: int, timeout: float) -> None:
        """Open the serial port."""
        self.s = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self._logger = logging.getLogger(__name__)
        self._serial_lock = Lock()

    def ping(self, id: np.uint8) -> None:
        """Ping an Orbita with the given ID.
        
        Raises a TimeoutError if the ID does not respond.
        """
        _, errors = self._send_msg(id, bytearray([Instruction.PING.value]))
        return errors

    def read(self, id: np.uint8, register: OrbitaRegister) -> Tuple[Any, List[OrbitaError]]:
        """Get the register value of the specified Orbita.
        
        The return values are expressed in the format specified by OrbitaRegister. 
        A value for each motor is returned (disk order TODO).

        A READ_DATA instruction message is defined as:
        [0xff, 0xff, ORBITA_ID, 4, READ_DATA, REG_ADDR, REG_LENGTH, CRC]
        Please note that the REG_LENGTH is ignored and only used for Dynamixel protocol compatibility.

        A status packet is returned as:
        [0xff, 0xff, ORBITA_ID, 5 + NB_PARAM, ERR, PARAM1, PARAM2, ..., CRC]
        Where PARAM* is the returned value coded as defined by https://docs.python.org/3/library/struct.html#format-characters
        """
        addr, datafmt, perm = register.value
        msg, errors = self._send_msg(id, bytearray([Instruction.READ_DATA.value, addr, 0x01]))

        if 'r' not in perm:
            raise ValueError(f'Register "{register.name}" is write only!')

        payload = msg[5:-1]
        
        val = list(struct.unpack(3 * datafmt, payload))
        if len(datafmt) > 1:
            val = np.array(val).reshape(len(datafmt), -1).tolist()
        
        return errors, val

    def write(self, id: np.uint8, register: OrbitaRegister, values) -> List[OrbitaError]:
        """Set the register value of the specified Orbita.
        
        The given values must be expressed in the format specified by OrbitaRegister.
        A value for each motor must be given (disk order TODO).

        A WRITE_DATA instruction message is defined as:
        [0xff, 0xff, ORBITA_ID, 3 + NB_PARAM, WRITE_DATA, REG_ADDR, PARAM1, PARAM2, ..., CRC]

        A status packet is returned as:
        [0xff, 0xff, ORBITA_ID, 2, ERR, CRC]
        """

        addr, datafmt, perm = register.value

        if 'w' not in perm:
            raise ValueError(f'Register "{register.name}" is read only!')

        coded_values = struct.pack(
            datafmt * 3,
            *np.array(values).flatten().tolist()
        )
            
        _, errors = self._send_msg(id, bytearray([Instruction.WRITE_DATA.value, addr]) + coded_values)
        return errors

    def _send_msg(self, id: np.uint8, instr: bytes):
        with self._serial_lock:
            header = bytearray([0xff, 0xff, id, len(instr) + 1])
                
            msg = header + instr
            msg += bytearray([self._compute_crc(msg)])

            self._logger.debug('>>> "{}"'.format(' '.join([hex(x) for x in msg])))
                
            self.s.write(msg)
            
            header = self.s.read(OrbitaSerialIO.HEADER_SIZE)

            if len(header) < OrbitaSerialIO.HEADER_SIZE:
                raise TimeoutError(f'Did not get an answer from Orbita (id="{id}")!')

            _, _, id, length = header
            status = self.s.read(length)
            
            msg = header + status
            self._logger.debug('<<< "{}"'.format(' '.join([hex(x) for x in msg])))
                
            if self._compute_crc(msg[:-1]) != msg[-1]:
                return msg, [OrbitaError.Checksum]

            errors = OrbitaError.from_error_code(msg[4])
            if errors:
                self._logger.warning(f'Got error {[err for err in errors]} on Orbita "{id}"!')
                        
            return msg, errors

    def _compute_crc(self, msg: bytes) -> np.uint8:
        return 255 - sum(msg[2:]) % 256


