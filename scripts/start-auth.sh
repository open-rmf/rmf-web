#!/bin/sh
set -e

# checks if user is in "docker" group, if so, we don't need to run as root.
cmd="docker-compose -f $(realpath $(dirname $0))/../e2e/keycloak/docker-compose.yml up"
if groups | grep -q '\bdocker\b'; then
  $cmd
else
  # execute command as root, we can't use sudo as we don't get an interactive tty. "pkexec" is the
  # gui alternative that prompts for a password in a popup.
  pkexec $cmd
fi
