#!/bin/bash
set -e

if [[ $# != 1 ]]; then
  echo 'Usage generate-models.sh <rmf-branch-or-tag>'
  exit 1
fi

rmf_branch=$1
script_dir=$(realpath $(dirname $0))
cd "$script_dir"

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

if ros2 pkg list | grep "${grep_args[@]}"; then
  echo 'One or more rmf packages is already found in your env, you probably have rmf sourced.'
  echo 'This may cause problems when generating the models, please run this script in a new terminal without rmf sourced.'
  exit 1
fi

rm -rf build
mkdir -p "$script_dir/build/rmf/src"
cd "$script_dir/build/rmf/src"
git clone --depth 1 -b "$rmf_branch" "https://github.com/open-rmf/rmf_internal_msgs.git"
git clone --depth 1 -b "$rmf_branch" "https://github.com/open-rmf/rmf_building_map_msgs.git"
. /opt/ros/foxy/setup.bash
cd ..
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
cd "$script_dir"

node generate-models.js "${rmf_msgs[@]}"

function getVersion {( set -e
  cd $1
  version=$(git tag --points-at HEAD)
  if [[ -z $version ]]; then
    version=$(git rev-parse HEAD)
  fi
  echo $version
)}

rmf_internal_msgs_version=$(getVersion "build/rmf/src/rmf_internal_msgs")
rmf_building_map_msgs_version=$(getVersion "build/rmf/src/rmf_building_map_msgs")
rmf_server_version=$(getVersion .)

cat << EOF > lib/version.ts
// THIS FILE IS GENERATED
export const version = {
  rmfInternalMsgs: '$rmf_internal_msgs_version',
  rmfBuildingMapMsgs: '$rmf_building_map_msgs_version',
  rmfServer: '$rmf_server_version',
};
EOF

echo ''
echo 'versions:'
echo "  rmf_internal_msgs: $rmf_internal_msgs_version"
echo "  rmf_building_map_msgs: $rmf_building_map_msgs_version"
echo "  rmf_server: $rmf_server_version"
echo ''
echo 'Successfully generated rmf-models'
