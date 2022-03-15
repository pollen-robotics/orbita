.. _HowToControl:

How to control Orbita?
======================

Orbita can be controlled in two ways:

- by directly sending target angle positions for each disk (in rads)
- by sending 3D desired orientation of the plate (as quaternion)

When using 3D orientation, the inverse kinematics solution will be computed to find the required disk positions.

.. image:: _static/orbita.gif
    :width: 50%
    :align: center

You can access:

- multi-turn current disk position in radians
- multi-turn target disk position in randians
- current disk orientation as quaternion
- target disk orientation as quaternion
- PID gains
- torque ON/OFF
- temperature
  
The internal PID controller runs at 1kHz.
