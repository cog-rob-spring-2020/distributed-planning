#!/usr/bin/env python
# DO NOT RUN THIS FILE. IT IS FOR ROS ONLY. RUNNING THIS WILL BREAK YOUR ROS INSTALLATION.

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     name='distributed_planning',
     version='0.0.1',
     description='The distributed planning package',
     packages=['distributed_planning',],
     package_dir={'': 'src',},
     install_requires={'numpy >= 1.16.0', "empy >= 3.3.4",},
)

setup(**setup_args)
