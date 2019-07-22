from distutils.core import setup

setup(
    name='spherical_joint',
    version='0.1.0',
    author='Pollen Robotics',
    packages=['spherical_joint'],
    url='https://github.com/pollen-robotics/spherical-joint',
    license='LICENSE.txt',
    description='Library used to control the spherical actuator',
    install_requires=[
        "numpy",
        "pyquaternion"
        ]
)
