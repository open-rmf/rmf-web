import os

import uvicorn
from uvicorn.config import LOGGING_CONFIG

from .app import app
from .app_config import app_config, load_config

app_config = load_config(
    os.environ.get(
        "RMF_API_SERVER_CONFIG",
        f"{os.path.dirname(__file__)}/default_config.py",
    )
)

TORTOISE_ORM = app_config.get_tortoise_orm_config()


def main():
    LOGGING_CONFIG["formatters"]["access"][
        "fmt"
    ] = '%(asctime)s %(levelprefix)s %(client_addr)s - "%(request_line)s" %(status_code)s'
    uvicorn.run(
        app,
        host=app_config.host,
        port=app_config.port,
        root_path=app_config.public_url.path,
        log_level=app_config.log_level.lower(),
    )


if __name__ == "__main__":
    main()
