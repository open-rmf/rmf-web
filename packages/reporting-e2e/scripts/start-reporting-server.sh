#!/bin/bash
set -e;

export PIPENV_PIPFILE=../../Pipfile

pipenv run reporting_server
