#!/bin/bash

set -o errexit
set -o nounset
set -o pipefail

SCRIPTPATH="$(
    cd -- "$(dirname "$0")" >/dev/null 2>&1
    pwd -P
)"
KEYCLOAK_PATH="$SCRIPTPATH/../.keycloak"

: "${ROOT_URL:=http://localhost:8080/auth}"
: "${KEYCLOAK_ADMIN:=admin}"
: "${KEYCLOAK_ADMIN_PASSWD:=admin}"
: "${PRODUCTION:=false}"

mkdir -p "$KEYCLOAK_PATH"

sudo apt install default-jre jq

echo "Checking if Keycloak executables have already been downloaded.."
[[ -f "$KEYCLOAK_PATH/bin/standalone.sh" ]] || \
    ( echo "Keycloak executable not found. Downloading.." && \
    wget https://github.com/keycloak/keycloak/releases/download/15.0.2/keycloak-15.0.2.tar.gz -O /tmp/keycloak.tar.gz && \
    tar -xzvf /tmp/keycloak.tar.gz -C "$KEYCLOAK_PATH" --strip-components=1 )


echo "Adding default admin"
bash "$KEYCLOAK_PATH/bin/add-user-keycloak.sh" -r master -u admin -p admin || true

bash "$KEYCLOAK_PATH/bin/standalone.sh"

