#!/bin/bash
set -e;

RMF_API_SERVER_CONFIG=rmf_server_config.py
PIPENV_PIPFILE=../../../Pipfile

pipenv run rmf_api_server
