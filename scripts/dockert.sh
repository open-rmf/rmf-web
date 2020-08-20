#!/bin/sh

# dockert = short for docker-trampoline.

# simple script that runs a command with "pkexec" if user is not in "docker" group.
# this is needed because we don't have a tty for sudo to work. "pkexec" runs a command as root and
# prompts for the password in a gui.
if groups | grep -q '\bdocker\b'; then
  docker "$@"
else
  pkexec docker "$@"
fi
