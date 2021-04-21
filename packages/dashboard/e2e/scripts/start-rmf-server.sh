#!/bin/bash
set -e;

export RMF_API_SERVER_CONFIG=rmf_server_config.py
export PIPENV_PIPFILE=../../../Pipfile
export RMF_SERVER_USE_SIM_TIME=true

pipenv run rmf_api_server
