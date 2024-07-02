#!/bin/bash
set -e
shopt -s globstar

HERE="$(dirname "$0")"

RMF_BUILDING_MAP_MSGS_VER=e26cf73ec7b1f61bbd450a4f85450e0db20d6c72
RMF_INTERNAL_MSGS_VER=e3f2dc688dcba79d2def064bae542fa0cadfd4dc
RMF_API_MSGS_VER=0141ab64e03bf08aba79be76b31ea02a55a3b296
RMF_ROS2_VER=0180dc78ef7b97266d873686d93deca12602413d
CODEGEN_VER=$($HERE/../../.venv/bin/datamodel-codegen --version)

cd "$HERE"

if [[ $ROS_DISTRO != 'jazzy' ]]; then
  echo 'Unable to find ros jazzy, please make sure that it is sourced.'
  exit 1
fi

function fetch_sources {
  url=$1
  commit=$2
  outdir=$3
  if [[ ! -d $outdir ]]; then
    git init "$outdir"
  fi
  pushd "$outdir"
  git fetch --depth=1 "$url" "$commit"
  git checkout "$commit"
  popd
}

fetch_sources https://github.com/open-rmf/rmf_building_map_msgs.git $RMF_BUILDING_MAP_MSGS_VER build/colcon_ws/src/rmf_building_map_msgs
fetch_sources https://github.com/open-rmf/rmf_internal_msgs.git $RMF_INTERNAL_MSGS_VER build/colcon_ws/src/rmf_internal_msgs
fetch_sources https://github.com/open-rmf/rmf_api_msgs.git $RMF_API_MSGS_VER build/rmf_api_msgs
fetch_sources https://github.com/open-rmf/rmf_ros2.git $RMF_ROS2_VER build/rmf_ros2

# build and source colcon workspace
pushd "build/colcon_ws"
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
popd

# generate rmf data models from ros messages
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
rm -rf api_server/models/ros_pydantic
pipenv run ros_translator -t=pydantic -o=api_server/models/ros_pydantic "${rmf_msgs[@]}"

cat << EOF > api_server/models/ros_pydantic/version.py
# THIS FILE IS GENERATED
version = {
  "rmf_internal_msgs": "$RMF_INTERNAL_MSGS_VER",
  "rmf_building_map_msgs": "$RMF_BUILDING_MAP_MSGS_VER",
}
EOF

pipenv run isort api_server/models/ros_pydantic
pipenv run black api_server/models/ros_pydantic

generate_from_json_schema() {
  input=$1
  output=$2
  upstream=$3 # the name of the package that contains the schemas
  version=$4 # the version of the upstream package

  rm -rf "$output"
  mkdir -p "$output"
  pipenv run datamodel-codegen \
    --disable-timestamp \
    --input-file-type jsonschema \
    --enum-field-as-literal one \
    --output-model-type pydantic_v2.BaseModel \
    --use-default-kwarg \
    --input "$input" --output "$output"
  cat << EOF > "$output/version.py"
# THIS FILE IS GENERATED
version = {
  "$upstream": "$version",
  "datamodel-code-generator": "${CODEGEN_VER}",
}
EOF
  pipenv run isort "$output"
  pipenv run black "$output"
}

# generate rmf api models from json schemas
generate_from_json_schema build/rmf_api_msgs/rmf_api_msgs/schemas api_server/models/rmf_api rmf_api_msgs $RMF_API_MSGS_VER
# there is a bug/conflict in RobotState.commission where the field name is same as the import name
sed -i -E '0,/(import )commission/s//\1commission as robot_commission/ ; s/commission.Commission/robot_commission.Commission/' api_server/models/rmf_api/robot_state.py

# generate builtin task descriptions from json schemas
generate_from_json_schema build/rmf_ros2/rmf_fleet_adapter/schemas api_server/models/rmf_ros2 rmf_ros2 $RMF_ROS2_VER

echo ''
echo 'versions:'
echo "  rmf_internal_msgs: $RMF_INTERNAL_MSGS_VER"
echo "  rmf_building_map_msgs: $RMF_BUILDING_MAP_MSGS_VER"
echo "  rmf_api_msgs: $RMF_API_MSGS_VER"
echo "  rmf_ros2: $RMF_ROS2_VER"
echo "  datamodel-code-generator: ${CODEGEN_VER}"
echo ''
echo 'Successfully generated ros_pydantic models'
