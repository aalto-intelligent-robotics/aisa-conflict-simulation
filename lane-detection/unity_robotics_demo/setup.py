from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['unity_robotics_demo'],
    package_dir={'': 'scripts'}
)

setup(**d)