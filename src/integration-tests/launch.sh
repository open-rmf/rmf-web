#!/bin/bash
set -e
set -x

ros2 launch demos office.launch.xml &
RMF_DEMO=$!
ros2 launch visualizer server.xml &
VIS_SERVER=$!

# FIXME: how to know if its ready to receive request?
sleep 5
ros2 launch demos office_loop.launch.xml

node node_modules/.bin/react-scripts "$@"

# ros2 launch only cleans up children with SIGINT, SIGTERM doesn't work
kill -SIGINT $RMF_DEMO
kill -SIGINT $VIS_SERVER
