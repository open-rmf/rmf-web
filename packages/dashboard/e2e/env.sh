#!/bin/bash

export TS_NODE_PROJECT=${TS_NODE_PROJECT:-tsconfig.json}
export REACT_APP_TRAJECTORY_SERVER=${REACT_APP_TRAJECTORY_SERVER:-ws://localhost:8006}
export REACT_APP_API_SERVER=${REACT_APP_API_SERVER:-ws://localhost:50002}
export ROMI_DASHBOARD_PORT=${ROMI_DASHBOARD_PORT:-5000}
export COMPOSE_PROJECT_NAME=${COMPOSE_PROJECT_NAME:-romidashboarde2e}

if [ -z "${CI}" ]; then
  echo "In local environment"
else
  export REACT_APP_AUTH_CONFIG='{"realm":"master", "clientId":"romi-dashboard", "url":"http://172.16.0.2:8080/auth"}'
fi
