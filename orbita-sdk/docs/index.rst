.. Orbita SDK documentation master file, created by
   sphinx-quickstart on Tue Mar  1 15:49:52 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.


Orbita's docs
=============

Table of Contents
-----------------

.. toctree:: 
   :maxdepth: 2

   control
   advanced_topic

Introduction
------------

Orbita is a 3D parallel joint designed by `Pollen-Robotics <https://pollen-robotics.com>`_.

This actuator was developped to:

- provide **full control of the plate** at the top - 3D rotation on a same point,
- and combine the torque of the **three motors used in parallel** for efficiency.

.. image:: _static/orbita_frame.jpg
   :width: 35%
   :align: center

The Orbita actuator embeds an electronic board that controls the three motors via a PID controller and gives access to temperature and extra information through :ref:`RS485 communication <RS485>`. 

It requires a 12V power supply.

This documentation will guide you in :ref:`HowToControl` using our :ref:`PythonSDK`. This will allow you to simply start make Orbita move. :ref:`AdvancedTopics` will also be approached.

For more information on the Orbita's design, you can check `this medium post <https://medium.com/pollen-robotics/orbita-is-turning-heads-literally-d10d378550e2>`_.
