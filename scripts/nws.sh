#!/bin/bash
set -e

usage() {
  echo 'Usage: nws.sh [--dependencies-only|-d] <script> <packages...>'
  echo ''
  echo 'nws.sh (shrot for npm workspace script) is a helper script to run npm scripts in npm workspace with proper dependency ordering.'
  echo 'As of npm 8.1.2, the --workspace argument in `npm run` only run scripts for the specified workspace,'
  echo 'for some workflow like build packages, we need to run depdentant scripts in the correct order in order for the target workspace to build correctly.'
}

# detects if we are already running from another nws script, if so, exit immediately so we don't
# run the same scripts multiple times
if [[ -n $nws_context ]]; then
  exit
fi
export nws_context=1

here=$(dirname "$0")

options=$(getopt -l'help' -o'h' -l'dependencies-only' -o'd' -- "$@")
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
    --dependencies-only | -d)
      dependencies_only='--dependencies-only'
      shift
      ;;
    --)
      shift
      break
      ;;
  esac
done

cmd=$1
shift
targets="$@"

eval npm --prefix "$(dirname $0)/.." run $cmd --if-present --include-workspace-root $(node "$here/workspace-args.js" $dependencies_only $(node $here/current-package.js))
