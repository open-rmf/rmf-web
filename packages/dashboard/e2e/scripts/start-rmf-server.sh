#!/bin/bash
set -e;

export RMF_API_SERVER_CONFIG=rmf_server_config.py
export PIPENV_PIPFILE=../../../Pipfile

pipenv run rmf_api_server
