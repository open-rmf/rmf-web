#!/bin/bash
set -e

pwd
cd "$(dirname $0)/.."

# Current version of poetry does not support system site packages so we manually create
# the venv before running poetry.
if [ ! -f .venv/pyvenv.cfg ]; then
  python3 -m venv --system-site-packages --prompt rmf-web .venv
fi

poetry install --no-root
