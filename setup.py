#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['action_server_monitor', 'action_server_monitor.widgets', 'action_server_monitor.goal_status'],
    package_dir={'': 'src'},
    scripts=['scripts/action_server_monitor']
)

setup(**d)
