import logging
import os
import sys

from fastapi import FastAPI
from rest_server.routers import log_router, report_router
from tortoise import Tortoise, run_async
from tortoise.contrib.fastapi import register_tortoise

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
    db_url="sqlite://:memory:",
    modules={"models": ["models.raw_log"]},
    generate_schemas=True,
    add_exception_handlers=True,
)
