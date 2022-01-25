#!/bin/bash
set -e
shopt -s globstar

RMF_BUILDING_MAP_MSGS_VER=c5e0352e2dfd3d11e4d292a1c2901cad867c1441
RMF_INTERNAL_MSGS_VER=0c237e1758872917661879975d7dc0acf5fa518c
RMF_API_MSGS_VER=2f20985a25279141fcb226402d67abfeb6db6944

cd "$(dirname $0)"
source ../../scripts/rmf-helpers.sh

check_rmf_not_sourced

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

# # generate rmf api models from json schemas
output='api_server/models/rmf_api'
rm -rf "$output"
mkdir -p "$output"
if [[ ! -d .venv_local/lib ]]; then
  python3 -m venv .venv_local
  . .venv_local/bin/activate && pip3 install wheel && pip3 install 'datamodel-code-generator~=0.11.15'
fi
. .venv_local/bin/activate && datamodel-codegen --disable-timestamp --input-file-type jsonschema --input build/rmf_api_msgs/rmf_api_msgs/schemas --output "$output"
cat << EOF > "$output/version.py"
# THIS FILE IS GENERATED
version = {
  "rmf_api_msgs": "$RMF_API_MSGS_VER",
}

EOF
pipenv run isort api_server/models/rmf_api
pipenv run black api_server/models/rmf_api

echo ''
echo 'versions:'
echo "  rmf_internal_msgs: $RMF_INTERNAL_MSGS_VER"
echo "  rmf_building_map_msgs: $RMF_BUILDING_MAP_MSGS_VER"
echo "  rmf_api_msgs: $RMF_API_MSGS_VER"
echo ''
echo 'Successfully generated ros_pydantic models'
