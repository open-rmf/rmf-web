# conflicts with isort because of local non-relative import
# pylint: disable=wrong-import-order

import logging
import os
import sys

from dependencies import auth_scheme, logger
from fastapi import Depends, FastAPI
from fastapi.middleware.cors import CORSMiddleware
from rest_server.routers import log_router, report_router
from tortoise.contrib.fastapi import register_tortoise

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


def get_app(fluentd_config=False):
    app = FastAPI()

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=False,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    if fluentd_config:
        app.include_router(log_router, prefix="/log", tags=["log"])

    if not fluentd_config:
        app.include_router(
            report_router,
            prefix="/report",
            tags=["report"],
            dependencies=[Depends(auth_scheme)],
        )

    register_tortoise(
        app,
        db_url=app_config.db_url,
        modules={"models": ["models"]},
        generate_schemas=True,
        add_exception_handlers=True,
    )

    return app
