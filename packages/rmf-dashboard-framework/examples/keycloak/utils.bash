#/usr/bin/bash

LOG_ERROR=1
__msg_error() {
    { [[ $LOG_ERROR == "1" ]] && echo -e "\e[31m[ERROR]:" "$@" "\e[0m" >&2; } || true
}

LOG_WARN=1
__msg_warn() {
    { [[ $LOG_WARN == "1" ]] && echo -e "\e[33m[WARN]:" "$@" "\e[0m" >&2; } || true
}

LOG_DEBUG=0
__msg_debug() {
    { [[ $LOG_DEBUG == "1" ]] && echo -e "\e[90m[DEBUG]:" "$@" "\e[0m" >&2; } || true
}

LOG_INFO=1
__msg_info() {
    { [[ $LOG_INFO == "1" ]] && echo -e "\e[34m[INFO]:" "$@" "\e[0m" >&2; } || true
}

__error_exit() {
    local line
    line="$1"
    shift
    __msg_error "$SCRIPT_NAME:$line" "$@"
    exit 1
}

KC_TOKEN=

kc_api() {
    local exit_code
    local resp
    __msg_debug kc_api ${*@Q}
    curl -ks --fail-with-body \
        -H "Content-Type: application/json" \
        -H "Authorization: Bearer $KC_TOKEN" \
        "$@"
}

# usage: kc_login <base_url> <user> <passwd>
kc_login() {
    local base_url=$1
    local user=$2
    local passwd=$3
    TOKEN_REQUEST_RESPONSE="$(curl -ks --fail-with-body -X POST \
        -H "Content-Type: application/x-www-form-urlencoded" \
        -d "username=$user" \
        -d "password=$passwd" \
        -d "grant_type=password" \
        -d "client_id=admin-cli" \
        "$base_url/realms/master/protocol/openid-connect/token" |
        jq -r '.access_token'
    )" || __msg_error "Failed to retrieve token, is Keycloak up?"
    if [[ "$TOKEN_REQUEST_RESPONSE" == "null" ]]; then
        echo 'Something went wrong retrieving token. Check credentials.'
    else
        KC_TOKEN="$TOKEN_REQUEST_RESPONSE"
        echo 'Successfully logged in to keycloak'
    fi
}
