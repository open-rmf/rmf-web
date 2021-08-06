#!/bin/bash
set -e

if [ $# -lt 1 ]; then
  echo 'usage: build-builder.sh <rmf-web-ws>'
fi

root_dir=$(realpath $(dirname $0))
cd $root_dir

[ -d build/builder/ ] && rm -r build/builder/
mkdir -p build/builder/

cd $1
cp lerna.json "$root_dir/build/builder/"
find . -name node_modules -prune -o -name build -prune -o \
  \(\
    -name 'package*.json' -o \
    -name 'Pipfile*' \
    -name 'setup.py' \
  \)\
  -exec cp --parents {} "$root_dir/build/builder/" \;
cp -r scripts/ "$root_dir/build/builder/"
cd $root_dir

cp docker/builder.Dockerfile build/builder/Dockerfile
docker build -t rmf-web/builder build/builder/
