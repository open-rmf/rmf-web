import os
from copy import deepcopy

from api_server.default_config import config as default_config

config = deepcopy(default_config)
config["jwt_public_key"] = "certs/keycloak.pub"
config[
    "oidc_url"
] = "http://localhost:8088/auth/realms/master/.well-known/openid-configuration"
config["aud"] = "rmf-dashboard"
config["iss"] = "http://localhost:8088/auth/realms/master"
