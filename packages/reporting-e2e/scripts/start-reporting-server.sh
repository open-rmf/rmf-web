#!/bin/bash
set -e;

export REPORTING_SERVER_CONFIG=reporting_server_config.py
export PIPENV_PIPFILE=../../Pipfile
export RMF_SERVER_USE_SIM_TIME=true

pipenv run reporting_server
