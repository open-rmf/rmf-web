import asyncio
import contextlib
import os
import signal
import threading
from typing import Any, Callable, Coroutine, Union

import schedule
from fastapi import Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.openapi.docs import (
    get_redoc_html,
    get_swagger_ui_html,
    get_swagger_ui_oauth2_redirect_html,
)
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
    User,
)
from .models import tortoise_models as ttm
from .repositories import TaskRepository
from .rmf_io import HealthWatchdog, RmfBookKeeper, rmf_events
from .types import is_coroutine


async def on_sio_connect(sid: str, _environ: dict, auth: dict | None = None) -> bool:
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


# will be called in reverse order on app shutdown
# TODO: convert them all to contexts
shutdown_cbs: list[Union[Coroutine[Any, Any, Any], Callable[[], None]]] = []


async def shutdown():
    while shutdown_cbs:
        cb = shutdown_cbs.pop()
        if is_coroutine(cb):
            await cb
        elif callable(cb):
            cb()

    logger.info("shutdown app")


@contextlib.asynccontextmanager
async def lifespan(_app: FastIO):
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
        task = loop.create_task(shutdown())
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

    logger.info("starting scheduler")
    asyncio.create_task(_spin_scheduler())
    scheduled_tasks = await ttm.ScheduledTask.all()
    scheduled = 0
    for t in scheduled_tasks:
        user = await User.load_from_db(t.created_by)
        if user is None:
            logger.warning(f"user [{t.created_by}] does not exist")
            continue
        task_repo = TaskRepository(user)
        await routes.scheduled_tasks.schedule_task(t, task_repo)
        scheduled += 1
    logger.info(f"loaded {scheduled} tasks")
    logger.info("successfully started scheduler")

    ros.spin_background()
    logger.info("started app")

    yield

    await shutdown()


app = FastIO(
    title="RMF API Server",
    lifespan=lifespan,
    socketio_connect=on_sio_connect,
    docs_url=None,
    redoc_url=None,
    separate_input_output_schemas=False,
)


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount(
    "/static",
    StaticFiles(
        directory=os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
    ),
    name="static",
)

os.makedirs(app_config.cache_directory, exist_ok=True)
app.mount(
    "/cache",
    StaticFiles(directory=app_config.cache_directory),
    name="cache",
)

rmf_bookkeeper = RmfBookKeeper(rmf_events, logger=logger.getChild("BookKeeper"))

app.include_router(routes.main_router)
app.include_router(
    routes.alerts_router, prefix="/alerts", dependencies=[Depends(user_dep)]
)
app.include_router(
    routes.beacons_router, prefix="/beacons", dependencies=[Depends(user_dep)]
)
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
    routes.scheduled_tasks.router,
    prefix="/scheduled_tasks",
    dependencies=[Depends(user_dep)],
)
app.include_router(
    routes.favorite_tasks_router,
    prefix="/favorite_tasks",
    dependencies=[Depends(user_dep)],
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


@app.get("/docs", include_in_schema=False)
async def custom_swagger_ui_html():
    openapi_url = f"{app_config.public_url.geturl()}{app.openapi_url}"
    return get_swagger_ui_html(
        openapi_url=openapi_url,
        title=app.title + " - Swagger UI",
        oauth2_redirect_url=app.swagger_ui_oauth2_redirect_url,
        swagger_js_url=f"{app_config.public_url.geturl()}/static/swagger-ui-bundle.js",
        swagger_css_url=f"{app_config.public_url.geturl()}/static/swagger-ui.css",
    )


@app.get(app.swagger_ui_oauth2_redirect_url, include_in_schema=False)
async def swagger_ui_redirect():
    return get_swagger_ui_oauth2_redirect_html()


@app.get("/redoc", include_in_schema=False)
async def redoc_html():
    openapi_url = f"{app_config.public_url.geturl()}{app.openapi_url}"
    return get_redoc_html(
        openapi_url=openapi_url,
        title=app.title + " - ReDoc",
        redoc_js_url=f"{app_config.public_url.geturl()}/static/redoc.standalone.js",
    )


async def _spin_scheduler():
    while True:
        schedule.run_pending()
        await asyncio.sleep(1)


async def _load_states():
    logger.info("loading states from database...")

    door_states = [DoorState.from_tortoise(x) for x in await ttm.DoorState.all()]
    for state in door_states:
        rmf_events.door_states.on_next(state)
    logger.info(f"loaded {len(door_states)} door states")

    door_health = [
        await DoorHealth.from_tortoise_orm(x) for x in await ttm.DoorHealth.all()
    ]
    for health in door_health:
        rmf_events.door_health.on_next(health)
    logger.info(f"loaded {len(door_health)} door health")

    lift_states = [LiftState.from_tortoise(x) for x in await ttm.LiftState.all()]
    for state in lift_states:
        rmf_events.lift_states.on_next(state)
    logger.info(f"loaded {len(lift_states)} lift states")

    lift_health = [
        await LiftHealth.from_tortoise_orm(x) for x in await ttm.LiftHealth.all()
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
        await DispenserHealth.from_tortoise_orm(x)
        for x in await ttm.DispenserHealth.all()
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
        await IngestorHealth.from_tortoise_orm(x)
        for x in await ttm.IngestorHealth.all()
    ]
    for health in ingestor_health:
        rmf_events.ingestor_health.on_next(health)
    logger.info(f"loaded {len(ingestor_health)} ingestor health")
