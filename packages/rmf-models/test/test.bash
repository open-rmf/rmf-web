#!/bin/bash
set -e

colcon build
. install/setup.bash
rm -rf out
cd ..
python3 -m ts_ros -o test/out ts_ros_test_msgs
cd test
npx ts-node -T node_modules/.bin/jasmine "$@"
