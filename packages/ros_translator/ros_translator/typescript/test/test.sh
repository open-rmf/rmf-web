#!/bin/bash
set -e

cd $(dirname $0)
rm -rf out
pipenv run python -m ros_translator -t=typescript -o=out ros_translator_test_msgs
echo 'test build'
node ./node_modules/.bin/tsc --noEmit
echo 'ok'
node ./node_modules/.bin/ts-node -T ./node_modules/.bin/jasmine "$@"
