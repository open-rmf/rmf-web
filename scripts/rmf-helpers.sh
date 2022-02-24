#!/bin/bash
set -e

rmf_msgs=(
  'rmf_building_map_msgs'
  'rmf_charger_msgs'
  'rmf_door_msgs'
  'rmf_lift_msgs'
  'rmf_dispenser_msgs'
  'rmf_ingestor_msgs'
  'rmf_fleet_msgs'
  'rmf_task_msgs'
)

# NOTE: This sources `/opt/ros/foxy/setup.bash``.
check_rmf_not_sourced() {
  if [[ -z $ROS_DISTRO ]]; then
    echo 'Unable to find ros, make sure a supported ros distro is sourced.'
    exit 1
  fi
  if ros2 pkg list | grep "^rmf_"; then
    echo 'One or more rmf packages is already found in your env, you probably have rmf sourced.'
    echo 'This may cause problems when generating the models, please run this script in a new terminal without rmf sourced.'
    exit 1
  fi
}

# Usage: build_and_source_rmf_msgs <rmf_tag>
build_and_source_rmf_msgs() {
  check_rmf_not_sourced

  rm -rf build
  mkdir -p build/rmf/src
  pushd build/rmf/src
  git clone --depth 1 -b "$1" "https://github.com/open-rmf/rmf_internal_msgs.git"
  git clone --depth 1 -b "$1" "https://github.com/open-rmf/rmf_building_map_msgs.git"
  cd ..
  colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  . install/setup.bash
  popd
}
