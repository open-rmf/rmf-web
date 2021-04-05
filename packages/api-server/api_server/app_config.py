import importlib
import os
from dataclasses import dataclass
from typing import Optional


@dataclass
class AppConfig:
    host: str
    port: int
    db_url: str
    root_path: str
    socket_io_path: str
    static_path: str
    static_directory: str
    log_level: str
    jwt_public_key: Optional[str]
    oidc_url: Optional[str]


def _load_config() -> AppConfig:
    if "RMF_API_SERVER_CONFIG" in os.environ:
        config_file = os.environ["RMF_API_SERVER_CONFIG"]
    else:
        config_file = f"{os.path.dirname(__file__)}/default_config.py"

    spec = importlib.util.spec_from_file_location("config", config_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return AppConfig(**module.config)


app_config = _load_config()
