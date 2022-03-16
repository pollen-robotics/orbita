.. _PythonSDK:

Python SDK
==========

We assume here that you already start and connect Orbita to your PC.

The SDK should work on any OS (Linux, Mac, Win) and require Python >= 3.6. 

You can install it via pip directly:

``$ pip install orbita_sdk``

Or, you can download the SDK from the `GitHub repository <https://github.com/pollen-robotics/orbita/tree/RS485/orbita-sdk>`_.

.. warning:: If cloning the source, make sure you are on the RS485 branch!

Getting started
===============

The SDK lets you control an Orbita 3 dof actuator in Python.

It more details, with the SDK you can:

- **enable/disable the torque** of all 3 motors
- get Orbita's current position - you can either read the 3 **disks position** (in rads) or the plate **3D orientation** (as quaternion)
- set a new target position - you can either use disk position (in rads) or 3D orientation of the plate (as quaternion)
- get/set PID gains (same for all 3 motors)
- get each motor temperature (in degree celsius)

Communication with Orbita is rather fast (about ~1ms per command). So you can **control it at a few hundred Hz** (get its position and send a new target). Internal **PID controller runs at 1kHz**.

.. warning:: Communication performance may vary a lot depending on your serial driver! In particular, if you are using Windows, make sure to decrease the driver latency to 1ms. On Linux/Mac OS default driver should work fine.


Examples
--------

The best way to get started in how to use the SDK is to dive in our simple demos. We provide ready to use examples as `Jupyter notebooks <https://jupyter.org>`_.

Demo notebooks:

1. A simple `Getting started notebook <notebooks/GettingStarted.ipynb>`_ that will guide you in your first steps in controlling Orbita.
2. An `Advanced demo notebook <notebooks/AdvancedControlDemo.ipynb>`_ with more complex control trajectory.
3. An `Interactive notebook <notebooks/InteractiveControl.ipynb>`_ that provides you with a widget to directly control Orbita's 3D orientation.

Benchmark and test:

4. A `benchmark notebook <notebooks/Benchmark.ipynb>`_ will test the serial communication to make sure everything works fine on your computer.
5. A `PID tuning example <notebooks/PIDTuning.ipynb>`_ to tune PID gains. For instance, if working with additional charge at the end of the actuator.

Finding your serial port
------------------------

TODO

APIs
----

The full API can be found in the :ref:`APIs` section below.



