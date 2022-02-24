import logging
import sys

from fastapi.logger import logger as _logger

from .app_config import app_config

logger: logging.Logger = _logger

handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
logger.setLevel(app_config.log_level)


def format_exception(exception: Exception):
    return logger.error(f"{type(exception).__name__}:{exception}")
