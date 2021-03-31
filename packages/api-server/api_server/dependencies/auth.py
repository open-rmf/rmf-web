from fastapi.security import OpenIdConnect

from ..app_config import app_config
from .logging import logger

if app_config.oidc_url is not None:
    auth_scheme = OpenIdConnect(openIdConnectUrl=app_config.oidc_url)
else:
    logger.warning("rest authentication is disabled")
    auth_scheme = lambda: None  # no authentication
