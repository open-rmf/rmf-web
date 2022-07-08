#!/bin/bash
set -e

pwd
cd "$(dirname $0)/.."

# Current version of poetry does not support system site packages so we manually create
# the venv before running poetry.
if [ ! -f .venv/pyvenv.cfg ]; then
  python3 -m venv --system-site-packages --prompt rmf-web .venv
  # built in versions of pip causes install issues on ubuntu20.04 (no idea why)
  .venv/bin/pip install -U pip
fi

poetry install
