Safety
------

Orbita's firmware provides basic safety feature, based on on the motor temperature, checked every second.

There are three levels:

- the **fan temperature threshold**: Above this temperature, the fan turns on (default value: 35°C).
- the **alarm temperature**: Above this value, the LED will start blinking and you will get error alarm when getting value from the actuator, this will results in warning log (default value: 45°C).
- the **shutdown temperature**: Above this value, the motor will turn off (torque off), the LED will stay on. You will need to wait for the motor to cool down before using it again (default value: 50°C).

.. warning:: You can freely modify these thresholds, yet this may result in damaging the motors ⚠️

.. warning:: Orbita firmware does not implement any torque limitation, so be careful with what you are doing ⚠️



