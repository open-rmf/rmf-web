#!/bin/bash
set -e

cd $(dirname $0)
rm -rf out
python -m ros_translator -t=typescript -o=out ros_translator_test_msgs
echo 'test build'
tsc --noEmit
echo 'ok'
ts-node -T ../../../node_modules/jasmine/bin/jasmine.js "$@"
