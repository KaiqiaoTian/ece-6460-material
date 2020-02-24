#!/bin/bash
cd /home/ros
set -e
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -y
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make -DCMAKE_BUILD_TYPE=Release tests
catkin_make -DCMAKE_BUILD_TYPE=Release run_tests -j1
catkin_test_results build/test_results
