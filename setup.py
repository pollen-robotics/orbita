from distutils.core import setup

setup(
    name='spherical-joint',
    version='0.1.0',
    author='Pollen Robotics',
    packages=['spherical-joint'],
    url='https://github.com/pollen-robotics/spherical-joint',
    license='LICENSE.txt',
    description='Library used to control the spherical actuator',
    install_requires=[
        "numpy",
        "quaternion",
        "math",
    ],
)
