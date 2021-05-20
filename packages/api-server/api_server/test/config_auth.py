import os

from api_server.default_config import config

config.update(
    {
        "jwt_public_key": f"{os.path.dirname(__file__)}/test.pub",
        "log_level": "CRITICAL",
        "aud": "localhost",
        "iss": "localhost",
    }
)
