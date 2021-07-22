import uvicorn

from .app import app
from .app_config import default_config


def main():
    uvicorn.run(
        app,
        host=default_config.host,
        port=default_config.port,
        root_path=default_config.public_url.path,
        log_level=default_config.log_level.lower(),
    )


if __name__ == "__main__":
    main()
