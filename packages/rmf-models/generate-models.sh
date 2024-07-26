#!/bin/bash
set -e

cd "$(dirname $0)"
source ../../scripts/version.sh
source ../../scripts/rmf-helpers.sh

usage() {
  echo "Usage: generate-models.sh"
  echo "Options:"
  echo "  --rmf-internal-msgs <tag-or-branch>"
  echo "  --rmf-building-map-msgs <tag-or-branch>"
  echo "  --rmf-server-ver <version> If not provided, use the git sha of the current commit"
}

options=$(getopt -o '' -l help,rmf-internal-msgs:,rmf-building-map-msgs:,rmf-server-ver: -- "$@")
eval set -- "$options"
while true; do
  case "$1" in
    --rmf-internal-msgs)
      shift
      rmf_internal_msgs=$1
      ;;
    --rmf-building-map-msgs)
      shift
      rmf_building_map_msgs=$1
      ;;
    --rmf-server-ver)
      shift
      rmf_server_ver=$1
      ;;
    --help)
      usage
      exit 1
      ;;
    --)
      shift
      break
      ;;
  esac
  shift
done

if [[ -z $rmf_internal_msgs ]]; then
  usage
  exit 1
fi
if [[ -z $rmf_building_map_msgs ]]; then
  usage
  exit 1
fi
if [[ -z $rmf_server_ver ]]; then
  rmf_server_ver=$(getVersion .)
fi

build_and_source_rmf_msgs "$rmf_internal_msgs" "$rmf_building_map_msgs"
node generate-models.js "${rmf_msgs[@]}"

cat << EOF > lib/version.ts
// THIS FILE IS GENERATED
export const version = {
  rmf_internal_msgs: '$rmf_building_map_msgs',
  rmf_building_map_msgs: '$rmf_building_map_msgs',
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
