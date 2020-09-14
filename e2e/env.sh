#!/bin/bash

export TS_NODE_PROJECT=e2e/tsconfig.json
export REACT_APP_SOSS_SERVER=wss://localhost:50001
export REACT_APP_TRAJECTORY_SERVER=ws://localhost:8006
export ROMI_DASHBOARD_PORT=5000
export COMPOSE_PROJECT_NAME=${COMPOSE_PROJECT_NAME:-romidashboarde2e} # underscores and hyphens doesn't seem to work for ubuntu 18.04's version of docker-compose
