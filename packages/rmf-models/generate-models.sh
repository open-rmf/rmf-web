#!/bin/bash
set -e

rmf_internal_msgs_ver='0c237e1758872917661879975d7dc0acf5fa518c'
rmf_building_map_msgs_ver='1.2.0'
rmf_ros2_ver='3be4edfce77e1a4ecb5ce99504e7696eb61658d9'

cd "$(dirname $0)"
. ../../.venv/bin/activate

rm -rf build

# git shallow clone single commit
git_clone() {
  mkdir -p "build/src/$1"
  pushd "build/src/$1"
  git init
  git remote add origin "https://github.com/open-rmf/$1"
  git fetch --depth 1 origin "$2"
  git checkout FETCH_HEAD
  popd  
}

git_clone rmf_internal_msgs "$rmf_internal_msgs_ver"
git_clone rmf_building_map_msgs "$rmf_building_map_msgs_ver"
git_clone rmf_ros2 "$rmf_ros2_ver"

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
rm -rf lib/ros
python -m ros_translator -t=typescript -o=lib/ros "${rmf_msgs[@]}"
pnpm exec prettier -w lib/ros
echo 'THIS DIRECTORY IS GENERATED, DO NOT EDIT!!' > lib/ros/GENERATED

rm -rf lib/task_descriptions
pnpm exec json2ts build/src/rmf_ros2/rmf_fleet_adapter/schemas/ lib/task_descriptions --cwd build/src/rmf_ros2/rmf_fleet_adapter/schemas/
echo 'THIS DIRECTORY IS GENERATED, DO NOT EDIT!!' > lib/task_descriptions/GENERATED

cat << EOF > lib/version.ts
// THIS FILE IS GENERATED
export const version = {
  rmf_internal_msgs: '$rmf_internal_msgs_ver',
  rmf_building_map_msgs: '$rmf_building_map_msgs_ver',
  rmf_ros2: '$rmf_ros2_ver',
};
EOF

echo ''
echo 'versions:'
echo "  rmf_internal_msgs: $rmf_internal_msgs_ver"
echo "  rmf_building_map_msgs: $rmf_building_map_msgs_ver"
echo "  rmf_ros2: $rmf_ros2_ver"
echo ''
echo 'Successfully generated rmf-models'
