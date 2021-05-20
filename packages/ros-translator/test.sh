#!/bin/bash
set -e

cd $(dirname $0)

pushd ros_translator/test
colcon build
source install/setup.bash
popd

ros_translator/typescript/test/test.sh
