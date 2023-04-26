# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/analys/ego_planner_swarm_ar/devel;/home/analys/lidar_ws/devel;/home/analys/ewok_ws/devel;/home/analys/RotorTM/devel;/home/analys/AirSim/ros/devel;/home/analys/landing_ws/devel;/home/analys/fpn_ws/devel;/home/analys/flightmare_ws/devel;/home/analys/geo_ws/devel;/home/analys/catkin_ws/devel;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/analys/lidar_ws/src/aruco_detect/devel/.private/catkin_tools_prebuild/env.sh')

output_filename = '/home/analys/lidar_ws/src/aruco_detect/build/catkin_tools_prebuild/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
