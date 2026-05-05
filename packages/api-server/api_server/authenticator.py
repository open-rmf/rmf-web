from typing import Any, Callable, Coroutine, Protocol

import jwt
import jwt.algorithms
from fastapi import Depends, HTTPException
from fastapi.security import OpenIdConnect

from .app_config import app_config
from .models import User


class AuthenticationError(Exception):
    pass


def _claims_grant_admin(claims: dict) -> bool:
    realm_roles = (claims.get("realm_access") or {}).get("roles") or []
    app_meta_roles = (claims.get("app_metadata") or {}).get("roles") or []
    return "superuser" in realm_roles or "superuser" in app_meta_roles


class Authenticator(Protocol):
    async def verify_token(self, token: str | None) -> User: ...

    def fastapi_dep(self) -> Callable[..., Coroutine[Any, Any, User] | User]: ...


class JwtAuthenticator:
    _algorithms = jwt.algorithms.get_default_algorithms()
    del _algorithms["none"]

    def __init__(
        self,
        key_or_secret: "jwt.algorithms.AllowedPublicKeys | str | bytes | None",
        aud: str,
        iss: str,
        *,
        oidc_url: str = "",
        jwks_url: str = "",
    ):
        """
        Authenticates with a JWT token, the client must send an auth params with
        a "token" key.
        :param key_or_secret: PEM bytes / public key for RS256/ES256, or a shared
            secret for HS256. Pass None when using `jwks_url`.
        :param jwks_url: if set, the verifier fetches signing keys from this
            JWKS endpoint and selects the right one by token `kid`. Used by
            Supabase projects with asymmetric JWT signing keys (the default
            since late 2025).
        """
        if not jwks_url and key_or_secret is None:
            raise ValueError("either key_or_secret or jwks_url is required")
        self.aud = aud
        self.iss = iss
        self.oidc_url = oidc_url
        self._key_or_secret = key_or_secret
        self._jwks_client = jwt.PyJWKClient(jwks_url) if jwks_url else None

    async def _get_user(self, claims: dict) -> User:
        # Identifier preference: keycloak's `preferred_username`, then standard
        # OIDC `email`, then the JWT `sub` (Supabase issues a UUID here).
        username = (
            claims.get("preferred_username") or claims.get("email") or claims.get("sub")
        )
        if not username:
            raise AuthenticationError(
                "expected one of 'preferred_username', 'email' or 'sub' claims to be present"
            )

        # `User.load_from_db` rejects names starting with '_'. Supabase UUIDs and
        # emails are safe; nothing to sanitize.
        user = await User.load_or_create_from_db(username)

        # Promote to admin if the IdP marks this user as a superuser. Supports
        # both keycloak-style realm roles and supabase-style app_metadata roles.
        if not user.is_admin and _claims_grant_admin(claims):
            await user.update_admin(True)

        return user

    async def verify_token(self, token: str | None) -> User:
        if not token:
            raise AuthenticationError("authentication required")
        try:
            if self._jwks_client is not None:
                signing_key = self._jwks_client.get_signing_key_from_jwt(token).key
            else:
                signing_key = self._key_or_secret
            claims = jwt.decode(
                token,
                signing_key,
                algorithms=list(self._algorithms),
                audience=self.aud,
                issuer=self.iss,
            )
            user = await self._get_user(claims)

            return user
        except jwt.InvalidTokenError as e:
            print(e)
            raise AuthenticationError(str(e)) from e

    def fastapi_dep(self) -> Callable[..., Coroutine[Any, Any, User] | User]:
        async def dep(
            auth_header: str = Depends(OpenIdConnect(openIdConnectUrl=self.oidc_url)),
        ):
            parts = auth_header.split(" ")
            if len(parts) != 2 or parts[0].lower() != "bearer":
                raise HTTPException(401, "invalid bearer format")
            try:
                return await self.verify_token(parts[1])
            except AuthenticationError as e:
                raise HTTPException(401, str(e)) from e

        return dep


_set_keys = sum(
    1
    for v in (app_config.jwt_public_key, app_config.jwt_secret, app_config.jwks_url)
    if v
)
if _set_keys > 1:
    raise ValueError("only one of jwt_public_key, jwt_secret or jwks_url must be set")
if not app_config.iss:
    raise ValueError("iss is required")
if not app_config.aud:
    raise ValueError("aud is required")

if app_config.jwks_url:
    authenticator = JwtAuthenticator(
        None,
        app_config.aud,
        app_config.iss,
        oidc_url=app_config.oidc_url or "",
        jwks_url=app_config.jwks_url,
    )
elif app_config.jwt_public_key:
    with open(app_config.jwt_public_key, "br") as f:
        authenticator = JwtAuthenticator(
            f.read(),
            app_config.aud,
            app_config.iss,
            oidc_url=app_config.oidc_url or "",
        )
elif app_config.jwt_secret:
    authenticator = JwtAuthenticator(
        app_config.jwt_secret,
        app_config.aud,
        app_config.iss,
        oidc_url=app_config.oidc_url or "",
    )
else:
    raise ValueError("one of jwt_public_key, jwt_secret or jwks_url is required")


user_dep = authenticator.fastapi_dep()
