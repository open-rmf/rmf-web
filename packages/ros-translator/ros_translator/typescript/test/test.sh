#!/bin/bash
set -e

cd $(dirname $0)
rm -rf out
pipenv run python -m ros_translator -t=typescript -o=out ros_translator_test_msgs
echo 'test build'
npx tsc --noEmit
echo 'ok'
npx ts-node -T ../../../node_modules/.bin/jasmine "$@"
