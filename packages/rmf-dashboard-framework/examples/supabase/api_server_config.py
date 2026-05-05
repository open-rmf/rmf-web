"""api-server config that points the JWT verifier at a Supabase project.

Required env vars:
  SUPABASE_PROJECT_URL   e.g. https://abcd1234.supabase.co

Optional env vars:
  SUPABASE_JWT_SECRET    Legacy HS256 shared secret. Only set this if your
                         Supabase project uses legacy JWT signing (rare for
                         projects created after late 2025). When unset, the
                         api-server fetches asymmetric public keys from the
                         project's JWKS endpoint, which is what new Supabase
                         projects use by default.
  SUPABASE_DB_URL        full postgres URL; if unset, local sqlite is used.
                         Use the "Session" pooler URL on free tier:
                         postgres://postgres.<ref>:<password>@aws-0-<region>.pooler.supabase.com:5432/postgres
"""

import os

from api_server.default_config import config

project_url = os.environ["SUPABASE_PROJECT_URL"].rstrip("/")

config.update(
    {
        "iss": f"{project_url}/auth/v1",
        "aud": "authenticated",
        "jwt_secret": None,
        "jwt_public_key": None,
        "jwks_url": f"{project_url}/auth/v1/.well-known/jwks.json",
        "oidc_url": f"{project_url}/auth/v1/.well-known/openid-configuration",
        # The first user that signs up with this email will be auto-promoted to
        # admin on login, regardless of supabase roles. Override via env if
        # desired.
        "builtin_admin": os.environ.get("RMF_BUILTIN_ADMIN", "admin@example.com"),
    }
)

# Legacy HS256 fallback. Only used if the project still has a shared JWT
# secret instead of asymmetric keys. Opt in by setting SUPABASE_LEGACY_HS256=1
# AND providing SUPABASE_JWT_SECRET — having SUPABASE_JWT_SECRET set alone
# is ignored, so half-edited .env files don't accidentally disable JWKS.
if os.environ.get("SUPABASE_LEGACY_HS256") == "1":
    config["jwks_url"] = None
    config["jwt_secret"] = os.environ["SUPABASE_JWT_SECRET"]

if db_url := os.environ.get("SUPABASE_DB_URL"):
    # tortoise-orm's postgres driver expects `postgres://` not `postgresql://`.
    config["db_url"] = db_url.replace("postgresql://", "postgres://", 1)
