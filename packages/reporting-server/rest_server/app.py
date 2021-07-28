# conflicts with isort because of local non-relative import
# pylint: disable=wrong-import-order

import logging
import os
import sys

from dependencies import auth_scheme, logger
from fastapi import Depends, FastAPI
from fastapi.middleware.cors import CORSMiddleware
from rest_server.database import setup_database
from rest_server.routers import log_router, report_router

from .app_config import SystemMode

logger = logging.getLogger("rest_app")
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
if "RMF_REST_SERVER_DEBUG" in os.environ:
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.INFO)

logger.info("started app")


def get_app(run_config=SystemMode.ALL):

    app = FastAPI()

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=False,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    if run_config in (SystemMode.ALL, SystemMode.FLUENTD):
        app.include_router(log_router, prefix="/log", tags=["log"])

    if run_config in (SystemMode.ALL, SystemMode.REPORT):
        app.include_router(
            report_router,
            prefix="/report",
            tags=["report"],
            dependencies=[Depends(auth_scheme)],
        )

    setup_database(
        app, generate_schemas=run_config in (SystemMode.ALL, SystemMode.FLUENTD)
    )

    return app
