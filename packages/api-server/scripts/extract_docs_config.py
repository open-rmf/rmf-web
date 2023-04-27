from os.path import dirname

from api_server.default_config import config

config.update(
    {
        "port": 8100,
    }
)
