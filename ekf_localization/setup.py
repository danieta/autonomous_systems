#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['ekf_localization', 'ekf_localization_ros'],
 package_dir={'ekf_localization': 'common/src/ekf_localization', 'ekf_localization_ros': 'ros/src/ekf_localization_ros'}
)

setup(**d)
