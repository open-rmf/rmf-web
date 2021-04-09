#!/bin/bash
set -e

cd $(dirname $0)
colcon build
. install/setup.bash
pushd ..
rm -rf out
python3 -m pipenv run python -m ts_ros -o test/out ts_ros_test_msgs
popd
echo 'test build'
node ../node_modules/.bin/tsc --noEmit
echo 'ok'
npx ts-node -T ../node_modules/.bin/jasmine "$@"
