#!/bin/bash
set -e

function usage() {
  echo "Usage: generate-models.sh"
  echo "Options:"
  echo "  --rmf-tag <tag-or-branch>"
  echo "  --rmf-server-ver <version>"
}

options=$(getopt -o '' -l rmf-tag:,rmf-server-ver: -- "$@")
eval set -- "$options"
while true; do
  case "$1" in
    --rmf-tag)
      shift
      rmf_tag=$1
      ;;
    --rmf-server-ver)
      shift
      rmf_server_ver=$1
      ;;
    --)
      shift
      break
      ;;
  esac
  shift
done

script_dir=$(realpath $(dirname $0))
cd "$script_dir"

source ../../scripts/version.sh

if [[ -z $rmf_tag ]]; then
  rmf_tag='main'
fi
if [[ -z $rmf_server_ver ]]; then
  rmf_server_ver=$(getVersion .)
fi

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

# rm -rf build
# mkdir -p "$script_dir/build/rmf/src"
# cd "$script_dir/build/rmf/src"
# git clone --depth 1 -b "$rmf_tag" "https://github.com/open-rmf/rmf_internal_msgs.git"
# git clone --depth 1 -b "$rmf_tag" "https://github.com/open-rmf/rmf_building_map_msgs.git"
cd "$script_dir/build/rmf"
# colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
cd "$script_dir"

node generate-models.js "${rmf_msgs[@]}"

rmf_internal_msgs_ver=$(getVersion "build/rmf/src/rmf_internal_msgs")
rmf_building_map_msgs_ver=$(getVersion "build/rmf/src/rmf_building_map_msgs")

cat << EOF > lib/version.ts
// THIS FILE IS GENERATED
export const version = {
  rmfInternalMsgs: '$rmf_internal_msgs_ver',
  rmfBuildingMapMsgs: '$rmf_building_map_msgs_ver',
  rmfServer: '$rmf_server_ver',
};

EOF

echo ''
echo 'versions:'
echo "  rmf_internal_msgs: $rmf_internal_msgs_ver"
echo "  rmf_building_map_msgs: $rmf_building_map_msgs_ver"
echo "  rmf_server: $rmf_server_ver"
echo ''
echo 'Successfully generated rmf-models'
