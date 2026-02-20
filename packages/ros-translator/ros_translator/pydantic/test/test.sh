#!/bin/bash
set -e

cd $(dirname $0)
realpath .
rm -rf out
python -m ros_translator -t=pydantic -o=out ros_translator_test_msgs
echo 'test build'
echo 'ok'
realpath .
pnpm exec pyright ros_translator/pydantic/test/out
