from enum import Enum

class OrbitaRegister(Enum):
    """Orbita accessible registers.

    Each register is defined by a triplet:

    - its address (np.uint8)
    - element format (as defined by https://docs.python.org/3/library/struct.html#format-characters)
    - nb of element
    - its permission (r, w, or rw)

    Please note that TemperatureShutdown, PID, Zero and Id are stored in Orbita's internal memory and will be kept even when you turn off the actuator.

    """
    TemperatureShutdown = (1, 'f', 1, 'rw')
    PresentPosition = (10, 'i', 3, 'r')
    GoalPosition = (20, 'i', 3, 'rw')
    TorqueEnable = (30, '?', 3, 'rw')
    PID = (31, 'fff', 1, 'rw')
    Temperature = (32, 'f', 3, 'r')
    Zero = (40, 'i', 3, 'rw')
    Id = (70, 'B', 1, 'rw')