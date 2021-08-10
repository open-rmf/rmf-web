from copy import deepcopy

from api_server.default_config import config as default_config

config = deepcopy(default_config)
config["host"] = "0.0.0.0"
config["port"] = 8000
config["db_url"] = "postgres://rmf-server:rmf-server@rmf-server-db/rmf-server"
config["public_url"] = "https://example.com/rmf/api/v1"
config["log_level"] = "INFO"
config["builtin_admin"] = "admin"
