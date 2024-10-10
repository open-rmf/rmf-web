from os.path import dirname

from api_server.default_config import config

here = dirname(__file__)
run_dir = f"{here}/run"

config.update(
    {
        "jwt_public_key": f"{here}/../../keycloak-example.pub",
        "jwt_secret": None,
        "iss": "http://localhost:8080/realms/rmf-web",
    }
)
