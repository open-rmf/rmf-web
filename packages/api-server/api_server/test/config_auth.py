import os
from copy import deepcopy

# pylint: disable=import-error
from api_server.default_config import config as default_config

config = deepcopy(default_config)
config["jwt_public_key"] = f"{os.path.dirname(__file__)}/test.pub"
