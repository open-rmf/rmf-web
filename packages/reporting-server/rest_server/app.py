import logging
import os
import sys
from fastapi import FastAPI
from tortoise.contrib.fastapi import register_tortoise

from rest_server.routers import log_router, report_router
from .app_config import app_config

logger = logging.getLogger("rest_app")
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
if "RMF_REST_SERVER_DEBUG" in os.environ:
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.INFO)

logger.info("started app")

app = FastAPI()

app.include_router(log_router, prefix="/log", tags=["log"])
app.include_router(report_router, prefix="/report", tags=["report"])

register_tortoise(
    app,
    db_url=app_config.db_url,
    modules={"models": ["models"]},
    generate_schemas=True,
    add_exception_handlers=True,
)
