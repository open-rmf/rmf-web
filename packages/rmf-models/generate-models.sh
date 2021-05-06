#!/bin/bash
set -e

cd "$(dirname $0)"
source ../../scripts/version.sh
source ../../scripts/rmf-helpers.sh

usage() {
  echo "Usage: generate-models.sh"
  echo "Options:"
  echo "  --rmf-tag <tag-or-branch>"
  echo "  --rmf-server-ver <version> If not provided, use the git sha of the current commit"
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

if [[ -z $rmf_tag ]]; then
  rmf_tag='main'
fi
if [[ -z $rmf_server_ver ]]; then
  rmf_server_ver=$(getVersion .)
fi

build_and_source_rmf_msgs "$rmf_tag"
node generate-models.js "${rmf_msgs[@]}"

rmf_internal_msgs_ver=$(getVersion "build/rmf/src/rmf_internal_msgs")
rmf_building_map_msgs_ver=$(getVersion "build/rmf/src/rmf_building_map_msgs")

cat << EOF > lib/version.ts
// THIS FILE IS GENERATED
export const version = {
  rmf_internal_msgs: '$rmf_internal_msgs_ver',
  rmf_building_map_msgs: '$rmf_building_map_msgs_ver',
  rmf_server: '$rmf_server_ver',
};

EOF

echo ''
echo 'versions:'
echo "  rmf_internal_msgs: $rmf_internal_msgs_ver"
echo "  rmf_building_map_msgs: $rmf_building_map_msgs_ver"
echo "  rmf_server: $rmf_server_ver"
echo ''
echo 'Successfully generated rmf-models'
