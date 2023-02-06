from typing import Any, Callable, Coroutine, Optional, Union

import jwt
from fastapi import Depends, HTTPException
from fastapi.security import OpenIdConnect

from .app_config import app_config
from .logger import logger
from .models import User


class AuthenticationError(Exception):
    pass


class JwtAuthenticator:
    def __init__(self, pem_file: str, aud: str, iss: str, *, oidc_url: str = ""):
        """
        Authenticates with a JWT token, the client must send an auth params with
        a "token" key.
        :param pem_file: path to a pem encoded certificate used to verify a token.
        """
        self.aud = aud
        self.iss = iss
        self.oidc_url = oidc_url
        with open(pem_file, "r", encoding="utf8") as f:
            self._public_key = f.read()

    async def _get_user(self, claims: dict) -> User:
        if not "preferred_username" in claims:
            raise AuthenticationError(
                "expected 'preferred_username' username claim to be present"
            )

        username = claims["preferred_username"]
        return await User.load_or_create_from_db(username)

    async def verify_token(self, token: Optional[str]) -> User:
        if not token:
            raise AuthenticationError("authentication required")
        try:
            claims = jwt.decode(
                token,
                self._public_key,
                algorithms=["RS256"],
                audience=self.aud,
                issuer=self.iss,
            )
            return await self._get_user(claims)
        except jwt.InvalidTokenError as e:
            raise AuthenticationError(str(e)) from e

    def fastapi_dep(self) -> Callable[..., Union[Coroutine[Any, Any, User], User]]:
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


class StubAuthenticator(JwtAuthenticator):
    def __init__(self):  # pylint: disable=super-init-not-called
        self._user = User(username="stub", is_admin=True)

    async def verify_token(self, token: Optional[str]) -> User:
        return self._user

    def fastapi_dep(self) -> Callable[..., Union[Coroutine[Any, Any, User], User]]:
        return lambda: self._user


if app_config.jwt_public_key:
    if app_config.iss is None:
        raise ValueError("iss is required")
    authenticator = JwtAuthenticator(
        app_config.jwt_public_key,
        app_config.aud,
        app_config.iss,
        oidc_url=app_config.oidc_url or "",
    )
else:
    authenticator = StubAuthenticator()
    logger.warning("authentication is disabled")

user_dep = authenticator.fastapi_dep()
