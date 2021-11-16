#!/bin/bash

eval npm --prefix "$(dirname $0)/.." run build $(node "$(dirname $0)/workspace-args.js" --only-direct "$@")
