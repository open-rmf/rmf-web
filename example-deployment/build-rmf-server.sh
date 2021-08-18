#!/bin/bash
set -e

if [ $# -lt 3 ]; then
  echo 'usage: build-rmf-server.sh <rmf-ws> <rmf-web-ws> <builder-tag>'
fi

root_dir=$(realpath $(dirname $(dirname $0)))
rmf_ws=$(realpath $1)
rmf_web_ws=$(realpath $2)
tag=$3

cd $root_dir

docker build -t rmf-web/rmf-server:build -f docker/rmf-server.build.Dockerfile --build-arg BUILDER_TAG=$tag "$rmf_web_ws"
docker build -t rmf-web/rmf-server:$tag -f docker/rmf-server.Dockerfile "$rmf_ws/src"
