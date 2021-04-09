from fastapi import Depends, HTTPException
from fastapi.security import OpenIdConnect

from ..app_config import app_config
from ..authenticator import AuthenticationError, JwtAuthenticator
from .logging import logger

if app_config.jwt_public_key:
    authenticator = JwtAuthenticator(app_config.jwt_public_key)

    if app_config.oidc_url:
        oidc_url = app_config.oidc_url
    else:
        oidc_url = ""

    # TODO: Authorization code flow doesn't actually work because of this bug https://github.com/swagger-api/swagger-ui/issues/6741.
    # althrough fastapi uses the latest version of swagger ui from a cdn,
    # the oauth2-redirect is hardcoded using an old version.
    #
    # Also, password flow may not work because it is not actually part of the oidc standard.
    # For keycloak, it doesn't return the id_token, which is required in a typical authorization code
    # flow.
    #
    # On top of the above problems, swagger ui doesn't actually fully support oidc in the
    # way we use it. We expect the id_token because it is the only token that MUST be a
    # JWT according to oidc spec (although most implementation nowadays also uses JWT
    # for the access token). Swagger UI passes the access_token to the server instead of the
    # id_token, so in the case that the authentication provider is not using a JWT for the
    # access_token, the authentication will fail. Even if the provider uses JWT for access_token,
    # it may still fail because of the different claims present in the access_token and id_token.
    # In the oidc spec, for an id_token, the "aud" claim MUST contain the client_id, but in
    # the case of keycloak, even though the access_token is also a JWT, it does not have to
    # follow the id_token spec, so the "aud" claim MAY NOT contain the client_id.
    # This causes authentication to fail because in the JWT spec, if the "aud" claim is present,
    # the verifier MUST check that it contains an expected value, since we expect an oidc
    # id_token, we MUST verify that the "aud" contains the client_id.
    security_scheme = OpenIdConnect(openIdConnectUrl=oidc_url)

    def auth_scheme(auth_header: str = Depends(security_scheme)):
        parts = auth_header.split(" ")
        if len(parts) != 2 or parts[0].lower() != "bearer":
            raise HTTPException(401, "invalid bearer format")
        try:
            authenticator.verify_token(parts[1])
        except AuthenticationError as e:
            raise HTTPException(401, str(e)) from e


else:
    logger.warning("authentication is disabled")
    authenticator = None
    auth_scheme = lambda: None  # no authentication
