#!/usr/bin/env bash
# Start the rmf api-server pointed at a cloud Supabase project.
#
# Usage:
#   # 1. populate .env (copy from .env.example) with SUPABASE_PROJECT_URL,
#   #    SUPABASE_JWT_SECRET, and optionally SUPABASE_DB_URL.
#   # 2. source the rmf workspace, then:
#   examples/supabase/run-api-server.bash

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${here}/../../../.." && pwd)"

if [[ -f "${here}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${here}/.env"
  set +a
fi

: "${SUPABASE_PROJECT_URL:?must be set (see examples/supabase/.env.example)}"
# SUPABASE_JWT_SECRET is only needed for legacy HS256 projects. New Supabase
# projects use asymmetric keys and the api-server fetches them from JWKS.

export RMF_API_SERVER_CONFIG="${here}/api_server_config.py"

exec pnpm -C "${repo_root}/packages/api-server" start
