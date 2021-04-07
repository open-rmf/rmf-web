from abc import ABC, abstractmethod
from typing import Optional

import jwt

from .app_config import app_config


class Authenticator(ABC):
    @abstractmethod
    def authenticate(self, environ: dict, auth: Optional[dict]):
        pass


class AuthenticationError(Exception):
    pass


class StubAuthenticator(Authenticator):
    """
    Authenticator that always authenticates successfully, i.e. no authentication.
    """

    def authenticate(self, environ: dict, auth: Optional[dict]):
        pass


class JwtAuthenticator(Authenticator):
    def __init__(self, pem_file: str):
        """
        Authenticates with a JWT token, the client must send an auth params with
        a "token" key.
        :param pem_file: path to a pem encoded certificate used to verify a token.
        """
        with open(pem_file, "br") as f:
            self._public_key = f.read()

    def authenticate(self, environ: dict, auth: Optional[dict]):
        """
        Raises error if token verification fails.
        """
        if auth is None:
            raise AuthenticationError("no auth options provided")
        if "token" not in auth:
            raise AuthenticationError("no token provided")
        token = auth["token"]
        try:
            jwt.decode(
                token,
                self._public_key,
                algorithms=["RS256"],
                audience=app_config.client_id,
            )
        except jwt.InvalidTokenError as e:
            raise AuthenticationError from e
