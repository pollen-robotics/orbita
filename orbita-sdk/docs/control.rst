.. _HowToControl:

How to control Orbita?
======================

Orbita can be controlled in two ways:

- by directly sending target angle positions for each disk (in rads)
- by sending 3D desired orientation of the plate (as quaternion)

When using 3D orientation, the analytical inverse kinematics solution will be computed to find the required disk positions.

.. note:: In this Orbita version, the inverse kinematics is not computed within the firmware but on the client computer (within the :ref:`PythonSDK`). Thus, the second option is only available through the Python's SDK at the moment.

.. image:: _static/orbita.gif
    :width: 50%
    :align: center

You can access:

- multi-turn current disk position (R)
- multi-turn target disk position (RW)
- PID gains (RW)
- torque ON/OFF (RW)
- temperature (R)
  
The internal PID controller runs at 1kHz.

The next steps:

.. toctree:: 
    :maxdepth: 1

    wiring
    safety
    zero
    sdk
    api

