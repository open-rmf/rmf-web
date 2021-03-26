import asyncio
import logging
import os
import sys

import socketio
from fastapi import FastAPI

from .app_config import app_config
from .tasks import tasks

logger = logging.getLogger("rest_app")
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
if "RMF_REST_SERVER_DEBUG" in os.environ:
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.INFO)

app = FastAPI()
sio = socketio.AsyncClient()

app.include_router(tasks.router)
logger.info("started app")
