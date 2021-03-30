import uvicorn

from .app import app
from .app_config import app_config


def main():
    uvicorn.run(
        app,
        host=app_config.host,
        port=app_config.port,
        log_level=app_config.log_level.lower(),
    )


if __name__ == "__main__":
    main()
