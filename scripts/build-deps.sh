#!/bin/bash

node "$(dirname $0)/workspace-args.js" --only-direct "$@" | xargs npm run build
