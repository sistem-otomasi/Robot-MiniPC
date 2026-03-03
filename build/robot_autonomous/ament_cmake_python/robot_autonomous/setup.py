from setuptools import find_packages
from setuptools import setup

setup(
    name='robot_autonomous',
    version='1.0.0',
    packages=find_packages(
        include=('robot_autonomous', 'robot_autonomous.*')),
)
