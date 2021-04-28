import importlib.util
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
    jwt_public_key: Optional[str]
    oidc_url: Optional[str]
    client_id: str

    def __post_init__(self):
        self.public_url = urllib.parse.urlparse(self.public_url)


def _load_config() -> AppConfig:
    if "RMF_REPORT_REST_SERVER_CONFIG" in os.environ:
        config_file = os.environ["RMF_REPORT_REST_SERVER_CONFIG"]
    else:
        config_file = f"{os.path.dirname(__file__)}/default_config.py"

    spec = importlib.util.spec_from_file_location("config", config_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return AppConfig(**module.config)


app_config = _load_config()
