#!/usr/bin/bash
set -e

cd "$(dirname "$0")/.."

if [ ! -d .venv ]; then
  echo 'creating virtualenv at .venv'
  python3 -m venv .venv --system-site-packages --prompt rmf-web
  .venv/bin/pip3 install pipenv --ignore-installed
fi
