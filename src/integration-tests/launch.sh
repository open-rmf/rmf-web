#!/bin/bash
set -e
set -x

function cleanup() {
  # ros2 launch doesn't kill children with SIGTERM, but works on SIGINT
  jobs -pr | xargs kill -SIGINT
  wait
}

trap cleanup EXIT

ros2 launch demos office.launch.xml > /dev/null &
ros2 launch visualizer server.xml > /dev/null &

# FIXME: how to know if its ready to receive request?
sleep 10
ros2 launch demos office_loop.launch.xml

node node_modules/.bin/react-scripts "$@"
