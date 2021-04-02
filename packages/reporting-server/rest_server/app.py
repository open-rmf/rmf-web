import asyncio
import logging
import os
import sys

from fastapi import FastAPI

# from .app_config import app_config

logger = logging.getLogger("rest_app")
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
if "RMF_REST_SERVER_DEBUG" in os.environ:
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.INFO)

app = FastAPI()


@app.get("/models")
async def on_startup():
    # connected = asyncio.Future()
    # sio.on("connect", lambda: connected.set_result(True))
    # await connected
    # del sio.handlers["/"]["connect"]

    app.include_router(
        prefix="/building_map",
    )

    logger.info("started app")
