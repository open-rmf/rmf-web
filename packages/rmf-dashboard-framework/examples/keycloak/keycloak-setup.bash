#!/bin/bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
PARSED=$(getopt -o 'u:o:' -n $SCRIPT_NAME -- "$@")
eval set -- "$PARSED"

KEYCLOAK_ADMIN=
KEY_OUT=
while true; do
    case "$1" in
        -u)
            KEYCLOAK_ADMIN="$2"
            shift 2
            ;;
        -o)
            KEY_OUT="$2"
            shift 2
            ;;
        *)
            shift
            break
    esac
done

print_usage() {
    echo "Usage: $SCRIPT_NAME -u <keycloak_admin> -o <public_key_output_path>"
}

if [[ ! $KEYCLOAK_ADMIN || ! $KEY_OUT ]]; then
    print_usage
    exit 1
fi

. "$(dirname "$0")/utils.bash"


: ${ROOT_URL:='http://localhost:5173'} # root url of the client
: ${KEYCLOAK_BASE_URL='http://localhost:8080'}

REALM_URL="$KEYCLOAK_BASE_URL/admin/realms"
REALM_CLIENT_URL="$REALM_URL/rmf-web/clients"
REALM_USERS_URL="$REALM_URL/rmf-web/users"
REALM_EVENTS_URL="$REALM_URL/rmf-web/events"
REALM_CLIENT_SCOPES_URL="$REALM_URL/rmf-web/client-scopes"

command -v jq >>/dev/null || { echo "Please install jq dependency (sudo apt install jq)"; exit 1; }

__msg_info "Retrieving JWT Token"
read -p "Enter password for $KEYCLOAK_ADMIN: " -s KEYCLOAK_ADMIN_PASSWD
kc_login "$KEYCLOAK_BASE_URL" "$KEYCLOAK_ADMIN" "$KEYCLOAK_ADMIN_PASSWD"> /dev/null

__msg_info "Creating rmf-web realm"
REALM_DATA="$(jq -cn '{
    "id":"rmf-web",
    "realm":"rmf-web",
    "enabled":"true"
}')"
# try to update the realm if it already exists, else create it
REALM_CREATION_RESPONSE=$(kc_api -X PUT \
    -d "$REALM_DATA" \
    "$REALM_URL/rmf-web") && exit_code=0 || exit_code=$?
if [[ $exit_code != 0 ]]; then
    __msg_info "Creating realm rmf-web"
    REALM_CREATION_RESPONSE=$(kc_api -X POST \
        -d "$REALM_DATA" \
        "$REALM_URL") && exit_code=0 || exit_code=$?
    if [[ $exit_code != 0 ]]; then
        __error_exit $LINENO "Failed to create realm"
    fi
    __msg_info "Realm rmf-web created"
else
    __msg_info "Realm already exists, skipping"
fi

__msg_info "Creating Admin User."
RMF_WEB_USER_ID=$(kc_api -X GET \
    "$REALM_USERS_URL?username=$KEYCLOAK_ADMIN" | jq -r ".[0].id") 
if [[ "$RMF_WEB_USER_ID" == "null" ]]; then
    RMF_WEB_USER_DATA="$(jq -cn --arg username "$KEYCLOAK_ADMIN" '{
        "username": $username,
        "enabled": true,
        "email": "\($username)@example.com",
        "emailVerified": true,
        "firstName": $username,
        "lastName": $username
    }')"
    USER_CREATION_RESPONSE=$(kc_api -X POST \
        -d "$RMF_WEB_USER_DATA" \
        "$REALM_USERS_URL") || __error_exit $LINENO "Failed to create admin user"
    RMF_WEB_USER_ID=$(kc_api -X GET \
        "$REALM_USERS_URL?username=$KEYCLOAK_ADMIN" | jq -r ".[0].id")
    RESET_PASSWORD_RESPONSE=$(kc_api -X PUT \
        -d '{"value": "'$KEYCLOAK_ADMIN_PASSWD'", "temporary": "false"}' \
        "$REALM_USERS_URL/$RMF_WEB_USER_ID/reset-password") || __error_exit $LINENO "Something went wrong resetting admin password."
    __msg_info "rmf-web user created"
else
    __msg_debug "rmf-web user already exists. Skipping."
fi

DASHBOARD_CLIENT_REQUEST_JSON=$(
    jq -cn \
        --arg rootUrl "$ROOT_URL" \
        --arg redirectRootUrl "$ROOT_URL/*" \
        '{
            "clientId": "dashboard",
            "rootUrl": $rootUrl,
            "redirectUris": [$redirectRootUrl],
            "webOrigins": [$rootUrl],
            "publicClient": true
        }'
)

__msg_info "Creating dashboard client"
DASHBOARD_CLIENT_ID=$(kc_api -X GET \
    "$REALM_CLIENT_URL?clientId=dashboard" | jq -r '.[0].id')
if [[ $DASHBOARD_CLIENT_ID == "null" ]]; then
    CLIENT_DASHBOARD_CREATION_RESPONSE=$(kc_api -X POST \
        -d "$DASHBOARD_CLIENT_REQUEST_JSON" \
        "$REALM_CLIENT_URL") || __error_exit $LINENO "Failed to create dashboard client"
    DASHBOARD_CLIENT_ID=$(kc_api -X GET \
        "$REALM_CLIENT_URL?clientId=dashboard" | jq -r '.[0].id')
    __msg_info "Created dashboard client"
else
    __msg_info "Found existing dashboard client. Updating instead."
    CLIENT_DASHBOARD_CREATION_RESPONSE=$(kc_api -X PUT \
        -d "$DASHBOARD_CLIENT_REQUEST_JSON" \
        "$REALM_CLIENT_URL/$DASHBOARD_CLIENT_ID") || __error_exit $LINENO "Failed to update dashboard client"
    __msg_info "Updated dashboard client"
fi

RMF_API_SERVER_CLIENT_SCOPE_DATA="$(jq -cn '{
  "name": "rmf_api_server",
  "protocol": "openid-connect",
  "description": "rmf_api_server audience scope"
}')"
__msg_info "Creating rmf api server client scope"
RMF_API_SERVER_CLIENT_SCOPE_ID=$(kc_api -X GET \
    "$REALM_CLIENT_SCOPES_URL" | jq -r '.[] | select ( .name == "rmf_api_server" ) | .id')
if [[ $RMF_API_SERVER_CLIENT_SCOPE_ID != "" ]]; then
    __msg_info "Found existing client scope, skipping"
else
    CLIENT_SCOPE_CREATION_RESPONSE=$(kc_api -X POST \
        -d "$RMF_API_SERVER_CLIENT_SCOPE_DATA" \
        "$REALM_CLIENT_SCOPES_URL") || __error_exit $LINENO "Failed to create client scope"
    RMF_API_SERVER_CLIENT_SCOPE_ID=$(kc_api -X GET \
        "$REALM_CLIENT_SCOPES_URL" | jq -r '.[] | select ( .name == "rmf_api_server" ) | .id')
    __msg_info "Created rmf api server client scope"
fi

RMF_API_SERVER_MAPPER_DATA="$(jq -cn '{
    "name": "rmf_api_server_audience",
    "protocol": "openid-connect",
    "protocolMapper": "oidc-audience-mapper",
    "consentRequired": false,
    "config": {
        "access.token.claim": "true",
        "id.token.claim": "false",
        "included.client.audience": "rmf_api_server"
    }
}')"
__msg_info "Creating rmf api server protocol mappers"
PROTOCOL_MAPPERS_RESP="$(kc_api -X GET "$REALM_CLIENT_SCOPES_URL/$RMF_API_SERVER_CLIENT_SCOPE_ID/protocol-mappers/models")"
RMF_API_SERVER_MAPPER_ID="$(echo $PROTOCOL_MAPPERS_RESP | jq -r '.[] | select ( .name == "rmf_api_server_audience" ) | .id')"
if [[ $RMF_API_SERVER_MAPPER_ID ]]; then
    __msg_debug "RMF_API_SERVER_MAPPER_ID=$RMF_API_SERVER_MAPPER_ID"
    __msg_info "Found existing rmf api server audience mapper, updating instead"
    MAPPER_DATA_WITH_ID="$(echo "$RMF_API_SERVER_MAPPER_DATA" | jq -c --arg mapper_id "$RMF_API_SERVER_MAPPER_ID" '. + {"id": $mapper_id}')"
    kc_api -X PUT \
        -d "$MAPPER_DATA_WITH_ID" \
        "$REALM_CLIENT_SCOPES_URL/$RMF_API_SERVER_CLIENT_SCOPE_ID/protocol-mappers/models/$RMF_API_SERVER_MAPPER_ID" > /dev/null
    __msg_info "Updated rmf api server protocol mappers"
else
    MAPPER_DATA_WITH_ID="$(echo "$RMF_API_SERVER_MAPPER_DATA" | jq -c --arg mapper_id "$RMF_API_SERVER_MAPPER_ID" '. + {"id": $mapper_id}')"
    kc_api -X POST \
        -d "$RMF_API_SERVER_MAPPER_DATA" \
        "$REALM_CLIENT_SCOPES_URL/$RMF_API_SERVER_CLIENT_SCOPE_ID/protocol-mappers/models" > /dev/null
    __msg_info "Created rmf api server protocol mappers"
fi

__msg_info "Linking up Client Scopes and Clients."
kc_api -X PUT \
    -d '{"value": "admin", "temporary": "false"}' \
    "$REALM_CLIENT_URL/$DASHBOARD_CLIENT_ID/default-client-scopes/$RMF_API_SERVER_CLIENT_SCOPE_ID" > /dev/null || \
        __error_exit $LINENO "Something went wrong assigning client scope."

__msg_info "Fetching token public key"

JWKS_URI=$(kc_api -X GET \
  "$KEYCLOAK_BASE_URL/realms/rmf-web/.well-known/openid-configuration" | jq -r '.jwks_uri')
__msg_debug "JWKS_URL=$JWKS_URI"

JWKS_X5C=$(kc_api -X GET \
  "$JWKS_URI" | jq -r '[ .keys[] | select(.use == "sig") ][0].x5c[0]')
__msg_debug "JWKS_X5C=$JWKS_X5C"

[[ -n "$JWKS_X5C" ]] || __error_exit $LINENO "Something went wrong trying to retrieve certificate."

PEM_FILE="-----BEGIN CERTIFICATE-----
$JWKS_X5C
-----END CERTIFICATE-----"
PUB_KEY=$(echo "$PEM_FILE" | openssl x509 -pubkey -noout)

__msg_info "Saving public key to $KEY_OUT"
echo -e "$PUB_KEY" > "$KEY_OUT"
