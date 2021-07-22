#!/bin/bash
set -e

cd "$(dirname $0)"
source ../../scripts/version.sh
source ../../scripts/rmf-helpers.sh

usage() {
  echo "Usage: generate-models.sh"
  echo "Options:"
  echo "  --rmf-tag <tag-or-branch>"
  echo "  --ros-translator-version <version> If not provided, use the git sha of the current commit"
}

options=$(getopt -o '' -l rmf-tag:,rmf-server-ver: -- "$@")
eval set -- "$options"
while true; do
  case "$1" in
    --rmf-tag)
      shift
      rmf_tag=$1
      ;;
    --ros-translator)
      shift
      ros_translator_ver=$1
      ;;
    --)
      shift
      break
      ;;
  esac
  shift
done

if [[ -z $rmf_tag ]]; then
  rmf_tag='main'
fi
if [[ -z $ros_translator_ver ]]; then
  ros_translator_ver=$(getVersion .)
fi

build_and_source_rmf_msgs "$rmf_tag"
pipenv run ros_translator -t=pydantic -o=task_scheduler/models/ros_pydantic "${rmf_msgs[@]}"

rmf_internal_msgs_ver=$(getVersion "build/rmf/src/rmf_internal_msgs")
rmf_building_map_msgs_ver=$(getVersion "build/rmf/src/rmf_building_map_msgs")

cat << EOF > task_scheduler/models/ros_pydantic/version.py
# THIS FILE IS GENERATED
version = {
  "rmf_internal_msgs": "$rmf_internal_msgs_ver",
  "rmf_building_map_msgs": "$rmf_building_map_msgs_ver",
  "ros_translator": "$ros_translator_ver",
}

EOF

pipenv run isort task_scheduler/models/ros_pydantic
pipenv run black task_scheduler/models/ros_pydantic

echo ''
echo 'versions:'
echo "  rmf_internal_msgs: $rmf_internal_msgs_ver"
echo "  rmf_building_map_msgs: $rmf_building_map_msgs_ver"
echo "  ros_translator: $ros_translator"
echo ''
echo 'Successfully generated ros_pydantic models'
