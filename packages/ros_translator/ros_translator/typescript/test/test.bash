#!/bin/bash
set -e

cd $(dirname $0)
colcon build
. install/setup.bash
pushd ..
rm -rf out
pipenv run python -m ros_translator -t=typescript -o test/out ts_ros_test_msgs
popd
echo 'test build'
node ./node_modules/.bin/tsc --noEmit
echo 'ok'
node ./node_modules/.bin/ts-node -T ./node_modules/.bin/jasmine "$@"
