from dependencies.logging import logger
from fastapi import Depends, HTTPException
from fastapi.security import HTTPBasic, HTTPBasicCredentials
from rest_server.app_config import app_config
from rest_server.authenticator import AuthenticationError, BasicAuthenticator

if app_config.jwt_public_key:
    authenticator = BasicAuthenticator()

    security = HTTPBasic()

    def basic_auth_scheme(credentials: HTTPBasicCredentials = Depends(security)):
        try:
            authenticator.verify_credentials(credentials)
        except AuthenticationError as e:
            raise HTTPException(401, str(e)) from e


else:
    logger.warning("basic authentication is disabled")
    authenticator = None

    def basic_auth_scheme():
        return None  # no authentication
