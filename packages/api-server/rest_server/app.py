import asyncio
import logging
import os
import sys

import socketio
from fastapi import FastAPI

from .app_config import app_config
from .building_map import building_map_router

# There is a bug with uvicorn and socketio that causes
# `RuntimeError: Event loop stopped before Future completed.`
# when getting SIGINT, the reason is because both uvicorn and socketio installs signal
# handlers, socketio's handler overwrites uvicorn's and in its handler to calls
# `asyncio.get_event_loop().stop()`.
# This forces the event loop to stop before uvicorn completes its work, hence the error appears.
# The side effect is that the shutdown handler is never called because the loop has
# been stopped.
#
# This is a hack to stop socketio from installing signal handlers.
socketio.asyncio_client.engineio.asyncio_client.async_signal_handler_set = True

logger = logging.getLogger("rest_app")
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
if "RMF_REST_SERVER_DEBUG" in os.environ:
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.INFO)

app = FastAPI()
sio = socketio.AsyncClient(logger=logger)


@app.on_event("startup")
async def on_startup():
    try:
        await sio.connect(app_config.api_server_url)
    except socketio.exceptions.ConnectionError:
        logger.error(
            "unable to connect to socketio server, some functions will not be available"
        )
        await sio.disconnect()

    if sio.connected:
        app.include_router(
            building_map_router(sio, logger.getChild("building_map")),
            prefix="/building_map",
        )

    logger.info("started app")


@app.on_event("shutdown")
async def on_shutdown():
    await sio.disconnect()
    logger.info("shutdown app")
