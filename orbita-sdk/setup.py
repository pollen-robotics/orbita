#!/usr/bin/env python

from setuptools import setup, find_packages


setup(
    name='orbita_sdk',
    version='0.1.0',
    packages=find_packages(),

    install_requires=[
        'future',
        'numpy',
        'pyserial',
        'pyquaternion',
        'scipy',
        'sklearn',
    ],

    extras_require={
        'doc': ['sphinx'],
    },

    package_data={'': ['mlpreg.obj']},

    author='Pollen Robotics',
    author_email='contact@pollen-robotics.com',
    description='Orbita actuator Python SDK.',
    url='https://github.com/pollen-robotics/orbita',
    license='Apache License 2.0',
)