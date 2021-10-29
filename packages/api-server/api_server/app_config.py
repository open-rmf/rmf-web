import importlib
import os
import urllib.parse
from dataclasses import dataclass
from typing import Optional


@dataclass
class AppConfig:
    host: str
    port: int
    db_url: str
    public_url: urllib.parse.ParseResult
    static_directory: str
    log_level: str
    builtin_admin: str
    jwt_public_key: Optional[str]
    oidc_url: Optional[str]
    aud: str
    iss: Optional[str]

    def __post_init__(self):
        self.public_url = urllib.parse.urlparse(self.public_url)

    def get_tortoise_orm_config(self):
        tortoise_config = {}
        tortoise_config["connections"] = {"default": self.db_url}
        tortoise_config["apps"] = {
            "models": {
                "models": ["api_server.models.tortoise_models", "aerich.models"],
                "default_connection": "default",
            },
        }

        return tortoise_config


def load_config(config_file: str) -> AppConfig:
    spec = importlib.util.spec_from_file_location("config", config_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = AppConfig(**module.config)
    if "RMF_API_SERVER_LOG_LEVEL" in os.environ:
        config.log_level = os.environ["RMF_API_SERVER_LOG_LEVEL"]
    return config
