import os
import secrets

import jwt
from fastapi.security import HTTPBasicCredentials

from .app_config import app_config


class AuthenticationError(Exception):
    pass


class JwtAuthenticator:
    def __init__(self, pem_file: str):
        """
        Authenticates with a JWT token, the client must send an auth params with
        a "token" key.
        :param pem_file: path to a pem encoded certificate used to verify a token.
        """
        with open(pem_file, "br") as f:
            self._public_key = f.read()

    def verify_token(self, token: str):
        try:
            jwt.decode(
                token,
                self._public_key,
                algorithms=["RS256"],
                audience=app_config.client_id,
            )
        except jwt.InvalidTokenError as e:
            raise AuthenticationError(str(e)) from e


class BasicAuthenticator:
    """
    Authenticates with a Basic authentication method, the client must send an auth params with
    a "user" and "password".
    """

    def verify_credentials(self, credentials: HTTPBasicCredentials):
        try:
            correct_username = secrets.compare_digest(
                credentials.username, os.environ["FLUENTD_USER"]
            )
            correct_password = secrets.compare_digest(
                credentials.password, os.environ["FLUENTD_USER"]
            )
        except Exception as e:
            raise AuthenticationError(str(e)) from e

        if not (correct_username and correct_password):
            raise AuthenticationError("Incorrect email or password")

        return credentials.username
