import logging
import os
import sys

from ..app_config import app_config

logger = logging.getLogger("app")
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
logger.setLevel(app_config.log_level)
if "RMF_API_SERVER_LOG_LEVEL" in os.environ:
    logger.setLevel(os.environ["RMF_API_SERVER_LOG_LEVEL"])
