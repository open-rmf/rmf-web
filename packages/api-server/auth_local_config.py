from copy import deepcopy

from api_server.default_config import config as default_config

config = deepcopy(default_config)

config["jwt_public_key"] = "/tmp/jwt-pub-key.pub"
config[
    "oidc_url"
] = "http://localhost:8080/auth/realms/rmf-web/.well-known/openid-configuration"
config["aud"] = "dashboard"
config["iss"] = "http://localhost:8080/auth/realms/rmf-web"
