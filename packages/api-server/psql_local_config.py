from copy import deepcopy

from api_server.default_config import config as default_config

config = deepcopy(default_config)

config["db_url"] = "postgres://postgres:postgres@127.0.0.1:5432"
