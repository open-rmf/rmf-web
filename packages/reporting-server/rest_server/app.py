import logging
import os
import sys

from fastapi import FastAPI
from routers import log, report

logger = logging.getLogger("rest_app")
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
if "RMF_REST_SERVER_DEBUG" in os.environ:
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.INFO)

app = FastAPI()

app.include_router(log.router, prefix="/log", tags=["log"])
app.include_router(report.router, prefix="/report", tags=["report"])

logger.info("started app")
