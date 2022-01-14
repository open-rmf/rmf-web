import importlib.util
import os
import urllib.parse
from dataclasses import dataclass
from importlib.abc import Loader
from typing import Any, Optional, cast


@dataclass
class AppConfig:
    host: str
    base_port: str
    log_level: str
    public_url: urllib.parse.ParseResult
    jwt_public_key: Optional[str]
    oidc_url: Optional[str]
    aud: Optional[str]
    iss: Optional[str]

    def __post_init__(self):
        self.public_url = urllib.parse.urlparse(cast(str, self.public_url))


def load_config(config_file: str) -> AppConfig:
    spec = importlib.util.spec_from_file_location("config", config_file)
    if spec is None:
        raise FileNotFoundError(f"Could not find config file '{config_file}'")
    module = importlib.util.module_from_spec(spec)
    loader = spec.loader
    if not isinstance(loader, Loader):
        raise RuntimeError("unable to load module")
    loader.exec_module(module)
    config = AppConfig(**cast(Any, module).config)
    if "RMF_TELEOP_SERVER_LOG_LEVEL" in os.environ:
        config.log_level = os.environ["RMF_TELEOP_SERVER_LOG_LEVEL"]
    return config


app_config = load_config(
    os.environ.get(
        "RMF_TELEOP_SERVER_CONFIG",
        f"{os.path.dirname(__file__)}/default_config.py",
    )
)
