#!/bin/bash
set -e

usage() {
  echo "Usage: bootstrap.sh [--help] <packages...>"
}

options=$(getopt -l'help' -o'h' -- "$@")
[ $? -eq 0 ] || { 
    usage
    exit 1
}
eval set -- "$options"
while true; do
  case "$1" in
    --help | -h)
      usage
      exit 1
      ;;
    --)
      shift
      break
      ;;
  esac
done
targets="$@"

if [[ -z "$C1" || "$CI" == 0 || "$CI" == 'false' ]]; then
  cmd='install'
else
  cmd='ci'
fi

# workaround https://github.com/npm/cli/issues/3208
mkdir -p "$(dirname "$0")"/../node_modules

eval npm --prefix "$(dirname $0)/.." $cmd --include-workspace-root $(node "$(dirname $0)/workspace-args.js" $targets)
