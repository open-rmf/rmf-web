#!/bin/bash
set -e

rmf_internal_msgs_ver='0c237e1758872917661879975d7dc0acf5fa518c'
rmf_building_map_msgs_ver='1.2.0'

cd "$(dirname $0)"
. ../../.venv/bin/activate

rm -rf build

mkdir -p build/rmf_internal_msgs
pushd build/rmf_internal_msgs
git init
git remote add origin https://github.com/open-rmf/rmf_internal_msgs
git fetch --depth 1 origin "$rmf_internal_msgs_ver"
git checkout FETCH_HEAD
popd

mkdir -p build/rmf_building_map_msgs
pushd build/rmf_building_map_msgs
git init
git remote add origin https://github.com/open-rmf/rmf_building_map_msgs
git fetch --depth 1 origin "$rmf_building_map_msgs_ver"
git checkout FETCH_HEAD
popd

pushd build
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
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to "${rmf_msgs[@]}"
popd

. build/install/setup.bash
rm -r lib/ros
python -m ros_translator -t=typescript -o=lib/ros "${rmf_msgs[@]}"
pnpm exec prettier -w lib/ros
echo 'THIS DIRECTORY IS GENERATED, DO NOT EDIT!!' > lib/ros/GENERATED

cat << EOF > lib/version.ts
// THIS FILE IS GENERATED
export const version = {
  rmf_internal_msgs: '$rmf_internal_msgs_ver',
  rmf_building_map_msgs: '$rmf_building_map_msgs_ver',
};
EOF

echo ''
echo 'versions:'
echo "  rmf_internal_msgs: $rmf_internal_msgs_ver"
echo "  rmf_building_map_msgs: $rmf_building_map_msgs_ver"
echo ''
echo 'Successfully generated rmf-models'
