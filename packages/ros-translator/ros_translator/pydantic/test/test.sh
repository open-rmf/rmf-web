#!/bin/bash
set -e

cd $(dirname $0)
rm -rf out
python -m ros_translator -t=pydantic -o=out ros_translator_test_msgs
echo 'test build'
echo 'ok'
pnpm exec pyright ros_translator/pydantic/test/out
