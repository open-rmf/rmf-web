from typing import Optional

from fastapi import Depends, HTTPException
from fastapi.security import OpenIdConnect

from ..authenticator import AuthenticationError, JwtAuthenticator


def auth_scheme(
    authenticator: Optional[JwtAuthenticator] = None, oidc_url: Optional[str] = None
):
    if not authenticator:
        return lambda: None  # no authentication

    oidc_url = oidc_url or ""

    security_scheme = OpenIdConnect(openIdConnectUrl=oidc_url)

    def dep(auth_header: str = Depends(security_scheme)):
        parts = auth_header.split(" ")
        if len(parts) != 2 or parts[0].lower() != "bearer":
            raise HTTPException(401, "invalid bearer format")
        try:
            authenticator.verify_token(parts[1])
        except AuthenticationError as e:
            raise HTTPException(401, str(e)) from e

    return dep
