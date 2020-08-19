#!/bin/sh

cmd="docker-compose -f $(realpath $(dirname $0))/../e2e/keycloak/docker-compose.yml up"
if groups | grep -q '\bdocker\b'; then
  $cmd
else
  pkexec $cmd
fi
