from setuptools import setup, find_packages
from os import path

here = path.abspath(path.dirname(__file__))

with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()


setup(
    name='orbita',
    version='0.3.1',

    description='Library used to control the Orbita actuator',
    long_description=long_description,
    long_description_content_type='text/markdown',

    url='https://github.com/pollen-robotics/orbita',
    author='Pollen Robotics',
    author_email='contact@pollen-robotics.com',

    packages=find_packages(exclude=['tests']),

    install_requires=[
        'numpy',
        'pyquaternion',
        ],
    extras_require={
        'test': ['scipy>=1.4.0'],
    },
)
