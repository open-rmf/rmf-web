#!/bin/bash
source /opt/ros/eloquent/setup.bash
cd rmf_demos_ws
CXX=g++-8 colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE --packages-ignore soss-ros2-test
