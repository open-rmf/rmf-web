from typing import Optional

from fastapi import Depends, HTTPException
from fastapi.security import OpenIdConnect

from ..authenticator import AuthenticationError, JwtAuthenticator
from ..models import User


def auth_scheme(
    client_id: str,
    authenticator: Optional[JwtAuthenticator] = None,
    oidc_url: Optional[str] = None,
):
    """
    Returns a tuple containing an authentication and user dependency. The authentication
    dependency should be applied app-wide or router-wide to verify a token. The user
    dependency should be applied on each endpoint that requires the user info.
    """
    if not authenticator:
        # no authentication
        return lambda: User(username="stub", roles=["_rmf_superadmin"])

    oidc_url = oidc_url or ""

    security_scheme = OpenIdConnect(openIdConnectUrl=oidc_url)

    def get_user(claims: dict):
        def _get_roles(claims: dict):
            if "resource_access" not in claims:
                return set()
            resource_access = claims["resource_access"]
            if client_id not in resource_access:
                return set()
            return set(resource_access[client_id]["roles"])

        if not "preferred_username" in claims:
            raise AuthenticationError(
                "expected 'preferred_username' username claim to be present"
            )
        return User(username=claims["preferred_username"], roles=_get_roles(claims))

    def dep(auth_header: str = Depends(security_scheme)):
        parts = auth_header.split(" ")
        if len(parts) != 2 or parts[0].lower() != "bearer":
            raise HTTPException(401, "invalid bearer format")
        try:
            claims = authenticator.verify_token(parts[1])
            return get_user(claims)
        except AuthenticationError as e:
            raise HTTPException(401, str(e)) from e

    return dep
