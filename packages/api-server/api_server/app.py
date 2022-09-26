import asyncio
import os
import signal
import threading
from typing import Any, Callable, Coroutine, List, Optional, Union

from fastapi import Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from tortoise import Tortoise

from . import gateway, ros, routes
from .app_config import app_config
from .authenticator import AuthenticationError, authenticator, user_dep
from .fast_io import FastIO
from .logger import logger
from .models import (
    DispenserHealth,
    DispenserState,
    DoorHealth,
    DoorState,
    IngestorHealth,
    IngestorState,
    LiftHealth,
    LiftState,
)
from .models import tortoise_models as ttm
from .repositories import StaticFilesRepository
from .rmf_io import HealthWatchdog, RmfBookKeeper, rmf_events
from .types import is_coroutine


async def on_sio_connect(sid: str, _environ: dict, auth: Optional[dict] = None):
    session = await app.sio.get_session(sid)
    token = None
    if auth:
        token = auth["token"]
    try:
        user = await authenticator.verify_token(token)
        session["user"] = user
        return True
    except AuthenticationError as e:
        logger.info(f"authentication failed: {e}")
        return False


app = FastIO(title="RMF API Server", socketio_connect=on_sio_connect)


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)
os.makedirs(app_config.static_directory, exist_ok=True)
app.mount(
    "/static",
    StaticFiles(directory=app_config.static_directory),
    name="static",
)

# will be called in reverse order on app shutdown
shutdown_cbs: List[Union[Coroutine[Any, Any, Any], Callable[[], None]]] = []

static_files_repo = StaticFilesRepository(
    f"{app_config.public_url.geturl()}/static",
    app_config.static_directory,
    logger.getChild("static_files"),
)

rmf_bookkeeper = RmfBookKeeper(rmf_events, logger=logger.getChild("BookKeeper"))

app.include_router(routes.main_router)
app.include_router(
    routes.building_map_router, prefix="/building_map", dependencies=[Depends(user_dep)]
)
app.include_router(
    routes.doors_router, prefix="/doors", dependencies=[Depends(user_dep)]
)
app.include_router(
    routes.lifts_router, prefix="/lifts", dependencies=[Depends(user_dep)]
)
app.include_router(
    routes.tasks_router, prefix="/tasks", dependencies=[Depends(user_dep)]
)
app.include_router(
    routes.dispensers_router, prefix="/dispensers", dependencies=[Depends(user_dep)]
)
app.include_router(
    routes.ingestors_router, prefix="/ingestors", dependencies=[Depends(user_dep)]
)
app.include_router(
    routes.fleets_router, prefix="/fleets", dependencies=[Depends(user_dep)]
)
app.include_router(
    routes.admin_router, prefix="/admin", dependencies=[Depends(user_dep)]
)
app.include_router(routes.internal_router, prefix="/_internal")


@app.on_event("startup")
async def on_startup():
    loop = asyncio.get_event_loop()

    await Tortoise.init(
        db_url=app_config.db_url,
        modules={"models": ["api_server.models.tortoise_models"]},
    )
    # FIXME: do this outside the app as recommended by the docs
    await Tortoise.generate_schemas()
    shutdown_cbs.append(Tortoise.close_connections())

    ros.startup()
    shutdown_cbs.append(ros.shutdown)

    gateway.startup()

    # shutdown event is not called when the app crashes, this can cause the app to be
    # "locked up" as some dependencies like tortoise does not allow python to exit until
    # it is closed "gracefully".
    def on_signal(sig, frame):
        task = loop.create_task(on_shutdown())
        if not loop.is_running():
            loop.run_until_complete(task)
        if sig == signal.SIGINT and callable(prev_sigint):
            prev_sigint(sig, frame)
        elif sig == signal.SIGTERM and callable(prev_sigterm):
            prev_sigterm(sig, frame)

    if threading.current_thread() is threading.main_thread():
        prev_sigint = signal.signal(signal.SIGINT, on_signal)
        prev_sigterm = signal.signal(signal.SIGTERM, on_signal)

    await ttm.User.update_or_create(
        {"is_admin": True}, username=app_config.builtin_admin
    )

    # Order is important here
    # 1. load states from db, this populate the sio/fast_io rooms with the latest data
    await _load_states()

    # 2. start the services after loading states so that the loaded states are not
    # used. Failing to do so will cause for example, book keeper to save the loaded states
    # back into the db and mess up health watchdog's heartbeat system.

    await rmf_bookkeeper.start()
    shutdown_cbs.append(rmf_bookkeeper.stop())
    health_watchdog = HealthWatchdog(
        rmf_events,
        logger=logger.getChild("HealthWatchdog"),
    )
    await health_watchdog.start()

    ros.spin_background()
    logger.info("started app")


@app.on_event("shutdown")
async def on_shutdown():
    while shutdown_cbs:
        cb = shutdown_cbs.pop()
        if is_coroutine(cb):
            await cb
        elif callable(cb):
            cb()

    logger.info("shutdown app")


async def _load_states():
    logger.info("loading states from database...")

    door_states = [DoorState.from_tortoise(x) for x in await ttm.DoorState.all()]
    for state in door_states:
        rmf_events.door_states.on_next(state)
    logger.info(f"loaded {len(door_states)} door states")

    door_health = [
        await DoorHealth.from_tortoise(x) for x in await ttm.DoorHealth.all()
    ]
    for health in door_health:
        rmf_events.door_health.on_next(health)
    logger.info(f"loaded {len(door_health)} door health")

    lift_states = [LiftState.from_tortoise(x) for x in await ttm.LiftState.all()]
    for state in lift_states:
        rmf_events.lift_states.on_next(state)
    logger.info(f"loaded {len(lift_states)} lift states")

    lift_health = [
        await LiftHealth.from_tortoise(x) for x in await ttm.LiftHealth.all()
    ]
    for health in lift_health:
        rmf_events.lift_health.on_next(health)
    logger.info(f"loaded {len(lift_health)} lift health")

    dispenser_states = [
        DispenserState.from_tortoise(x) for x in await ttm.DispenserState.all()
    ]
    for state in dispenser_states:
        rmf_events.dispenser_states.on_next(state)
    logger.info(f"loaded {len(dispenser_states)} dispenser states")

    dispenser_health = [
        await DispenserHealth.from_tortoise(x) for x in await ttm.DispenserHealth.all()
    ]
    for health in dispenser_health:
        rmf_events.dispenser_health.on_next(health)
    logger.info(f"loaded {len(dispenser_health)} dispenser health")

    ingestor_states = [
        IngestorState.from_tortoise(x) for x in await ttm.IngestorState.all()
    ]
    for state in ingestor_states:
        rmf_events.ingestor_states.on_next(state)
    logger.info(f"loaded {len(ingestor_states)} ingestor states")

    ingestor_health = [
        await IngestorHealth.from_tortoise(x) for x in await ttm.IngestorHealth.all()
    ]
    for health in ingestor_health:
        rmf_events.ingestor_health.on_next(health)
    logger.info(f"loaded {len(ingestor_health)} ingestor health")
