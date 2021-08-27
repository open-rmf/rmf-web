#!/bin/bash
set -e

rmf_msgs=(
  'rmf_task_msgs'
)

# Usage: build_and_source_rmf_msgs <rmf_tag>
build_and_source_rmf_msgs() {
  grep_args=()
  for pkg in ${rmf_msgs[@]}; do
    grep_args+=("-e$pkg")
  done

  . /opt/ros/foxy/setup.bash
  if ros2 pkg list | grep "${grep_args[@]}"; then
    echo 'One or more rmf packages is already found in your env, you probably have rmf sourced.'
    echo 'This may cause problems when generating the models, please run this script in a new terminal without rmf sourced.'
    exit 1
  fi
  rm -rf build
  mkdir -p build/rmf/src
  pushd build/rmf/src
  git clone --depth 1 -b "$1" "https://github.com/open-rmf/rmf_internal_msgs.git"
  cd rmf_internal_msgs/
  rm -rf rmf_charger_msgs rmf_door_msgs rmf_lift_msgs  rmf_ingestor_msgs rmf_fleet_msgs rmf_workcell_msgs rmf_traffic_msgs
  cd ../..
  colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  . install/setup.bash
  popd
}