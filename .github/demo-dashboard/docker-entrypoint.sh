#!/bin/sh
set -e

# Render the server urls into the dashboard at container start, so the same image
# works on any host. These urls are used by the browser, not by the container.
DASHBOARD_DIR="${DASHBOARD_DIR:-/opt/dashboard}"
INDEX_TEMPLATE="${INDEX_TEMPLATE:-/opt/index.html.template}"
RMF_SERVER_URL="${RMF_SERVER_URL:-http://localhost:8000}"
TRAJECTORY_SERVER_URL="${TRAJECTORY_SERVER_URL:-http://localhost:8006}"

# escape characters that are special in a sed replacement
escape() {
  printf '%s' "$1" | sed -e 's/[\\&|]/\\&/g'
}

sed \
  -e "s|__RMF_SERVER_URL__|$(escape "$RMF_SERVER_URL")|g" \
  -e "s|__TRAJECTORY_SERVER_URL__|$(escape "$TRAJECTORY_SERVER_URL")|g" \
  "$INDEX_TEMPLATE" > "$DASHBOARD_DIR/index.html"

echo "Dashboard using RMF_SERVER_URL=$RMF_SERVER_URL TRAJECTORY_SERVER_URL=$TRAJECTORY_SERVER_URL"

exec serve -sn "$DASHBOARD_DIR" "$@"
