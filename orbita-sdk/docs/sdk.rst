.. _PythonSDK:

Python's SDK
============

We assume here that you already start and connect Orbita to your PC.

The SDK should work on any OS (Linux, Mac, Win) and require Python >= 3.6. 

You can install it via pip directly:

``$ pip install orbita_sdk``

Or, you can download the SDK from the `GitHub repository <https://github.com/pollen-robotics/orbita/tree/RS485/orbita-sdk>`_.

.. warning:: If cloning the source, make sure you are on the RS485 branch!

SDK
---

.. automodule:: orbita_sdk.orbita
    :noindex:

The full API can be found in the :ref:`APIs` section below.


Examples
--------

We provide ready to use examples as `Jupyter notebooks <https://jupyter.org>`_.

Demo notebooks:

1. A simple `Getting started notebook <TODO>`_ that will guide you in your first steps in controlling Orbita.
2. An `Advanced demo notebook <TODO>`_ with more complex control trajectory.
3. An `Interactive notebook <TODO>`_ that provides you with a widget to directly control Orbita's 3D orientation.

Benchmark and test:

4. A `benchmark notebook <TODO>`_ will test the serial communication to make sure everything works fine on your computer.
5. A `PID tuning example <TODO>`_ to tune PID gains. For instance, if working with additional charge at the end of the actuator.


Finding your serial port
------------------------

.. .. - nettoyer + lien vers notebook de benchmark com