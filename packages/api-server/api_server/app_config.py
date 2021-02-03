import importlib
import os


class AppConfig():
    def __init__(self, dict):
        self.db_url: str = dict['db_url']
        self.public_url: str = dict['public_url']
        self.host: str = dict['host']
        self.port: int = dict['port']
        self.static_path: str = dict['static_path']
        self.static_directory: str = dict['static_directory']


def _load_config() -> AppConfig:
    if 'RMF_API_SERVER_CONFIG' in os.environ:
        config_file = os.environ['RMF_API_SERVER_CONFIG']
    else:
        config_file = f'{os.path.dirname(__file__)}/default_config.py'

    spec = importlib.util.spec_from_file_location('config', config_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return AppConfig(module.config)


app_config = _load_config()
