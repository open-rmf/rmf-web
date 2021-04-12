#!/bin/bash
set -e

function getVersion {( set -e
  cd $1
  version=$(git tag --points-at HEAD)
  if [[ -z $version ]]; then
    version=$(git rev-parse HEAD)
  fi
  echo $version
)}
