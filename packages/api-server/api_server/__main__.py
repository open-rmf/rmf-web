import logging
import os

import uvicorn
import uvicorn.logging
from uvicorn.config import LOGGING_CONFIG

from .app_config import load_config
from .logging import LogfmtFormatter, SafeLogfmtFormatter

app_config = load_config(
    os.environ.get(
        "RMF_API_SERVER_CONFIG",
        f"{os.path.dirname(__file__)}/default_config.py",
    )
)

handler = logging.StreamHandler()
handler.setFormatter(LogfmtFormatter())
logging.basicConfig(level=app_config.log_level, handlers=[handler])

# uvicorn access logs contains double quotes so we need to use the safer formatter
LOGGING_CONFIG["formatters"]["logfmt"] = {"()": SafeLogfmtFormatter}
LOGGING_CONFIG["handlers"]["logfmt"] = {
    "formatter": "logfmt",
    "class": "logging.StreamHandler",
    "stream": "ext://sys.stdout",
}
# uvicorn appears to log to both the default and configured handler,
# since we already configured the default logger,
# setting a handler here will cause double logging
LOGGING_CONFIG["loggers"]["uvicorn"]["handlers"] = []
LOGGING_CONFIG["loggers"]["uvicorn.access"]["handlers"] = ["logfmt"]


def main():
    # FIXME: we need to init logging before the app, better solution is to
    # NOT use global app that init on import
    from .app import app  # pylint: disable=import-outside-toplevel

    uvicorn.run(
        app,
        host=app_config.host,
        port=app_config.port,
        root_path=app_config.public_url.path,
        log_config=LOGGING_CONFIG,
        log_level=app_config.log_level.lower(),
    )


if __name__ == "__main__":
    main()
