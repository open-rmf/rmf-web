#!/bin/bash
set -e

pwd
cd "$(dirname $0)/.."

# bootstrap pipenv into the virtual environment
if [ ! -f .venv/pyvenv.cfg ]; then
  python3 -m venv --system-site-packages --prompt rmf-web .venv
  .venv/bin/pip install 'pipenv==2022.9.8'
fi

.venv/bin/pipenv install -d
