from enum import Enum

class OrbitaRegister(Enum):
    """Orbita accessible registers.

    Each register is defined by a triplet:

    - its address (np.uint8)
    - its format (as defined by https://docs.python.org/3/library/struct.html#format-characters)
    - its permission (r, w, or rw)

    """
    AngleLimit = (0, 'ii', 'rw')
    TemperatureShutdown = (1, 'f', 'rw')
    PresentPosition = (10, 'i', 'r')
    AbsolutePosition = (13, 'i', 'r')
    GoalPosition = (20, 'i', 'rw')
    TorqueEnable = (30, '?', 'rw')
    PID = (31, 'fff', 'rw')
    Temperature = (32, 'f', 'r')
    Zero = (40, 'i', 'rw')