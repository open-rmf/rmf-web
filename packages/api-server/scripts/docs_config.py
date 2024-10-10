import os

from api_server.default_config import config

here = os.path.dirname(__file__)

config.update(
    {
        "public_url": "http://localhost:8000/rmf-web",
    }
)
