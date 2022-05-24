import uvicorn

from .app import app
from .app_config import app_config


def main():
    uvicorn.run(
        app,
        host=app_config.host,
        port=app_config.port,
        root_path=app_config.public_url.path,
        log_level=app_config.log_level.lower(),
    )


if __name__ == "__main__":
    main()
