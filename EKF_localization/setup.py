#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['EKF_localization', 'EKF_localization_ros'],
 package_dir={'EKF_localization': 'common/src/EKF_localization', 'EKF_localization_ros': 'ros/src/EKF_localization_ros'}
)

setup(**d)
