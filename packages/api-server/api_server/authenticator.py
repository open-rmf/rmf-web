from typing import Any, Callable, Coroutine, Protocol

import jwt
import jwt.algorithms
from fastapi import Depends, HTTPException
from fastapi.security import OpenIdConnect

from .app_config import app_config
from .models import User


class AuthenticationError(Exception):
    pass


class Authenticator(Protocol):
    async def verify_token(self, token: str | None) -> User: ...

    def fastapi_dep(self) -> Callable[..., Coroutine[Any, Any, User] | User]: ...


class JwtAuthenticator:
    _algorithms = jwt.algorithms.get_default_algorithms()
    del _algorithms["none"]

    def __init__(
        self,
        key_or_secret: "jwt.algorithms.AllowedPublicKeys | str | bytes",
        aud: str,
        iss: str,
        *,
        oidc_url: str = "",
    ):
        """
        Authenticates with a JWT token, the client must send an auth params with
        a "token" key.
        :param pem_file: path to a pem encoded certificate used to verify a token.
        """
        self.aud = aud
        self.iss = iss
        self.oidc_url = oidc_url
        self._key_or_secret = key_or_secret

    async def _get_user(self, claims: dict) -> User:
        if not "preferred_username" in claims:
            raise AuthenticationError(
                "expected 'preferred_username' username claim to be present"
            )

        username = claims["preferred_username"]
        # FIXME(koonpeng): We should use the "userId" as the identifier. Some idP may allow
        # duplicated usernames.
        user = await User.load_or_create_from_db(username)

        return user

    async def verify_token(self, token: str | None) -> User:
        if not token:
            raise AuthenticationError("authentication required")
        try:
            claims = jwt.decode(
                token,
                self._key_or_secret,
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


if app_config.jwt_public_key and app_config.jwt_secret:
    raise ValueError("only one of jwt_public_key or jwt_secret must be set")
if not app_config.iss:
    raise ValueError("iss is required")
if not app_config.aud:
    raise ValueError("aud is required")

if app_config.jwt_public_key:
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
    raise ValueError("either jwt_public_key or jwt_secret is required")


user_dep = authenticator.fastapi_dep()
