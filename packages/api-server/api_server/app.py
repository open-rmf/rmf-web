import asyncio
import contextlib
import os
import signal
import threading
from typing import Any, Callable, Coroutine, Union

from fastapi import Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.openapi.docs import (
    get_redoc_html,
    get_swagger_ui_html,
    get_swagger_ui_oauth2_redirect_html,
)
from fastapi.staticfiles import StaticFiles
from tortoise import Tortoise

from api_server.repositories.cached_files import get_cached_file_repo
from api_server.rmf_io.events import (
    get_alert_events,
    get_beacon_events,
    get_fleet_events,
    get_rio_events,
    get_rmf_events,
    get_task_events,
)
from api_server.rmf_io.rmf_service import get_tasks_service
from api_server.scheduler import get_scheduler

from . import gateway, ros, routes
from .app_config import app_config
from .authenticator import AuthenticationError, authenticator, user_dep
from .fast_io import FastIO
from .logging import default_logger
from .models import DispenserState, DoorState, IngestorState, LiftState, User
from .models import tortoise_models as ttm
from .repositories import TaskRepository
from .rmf_io import RmfEvents
from .types import is_coroutine


async def on_sio_connect(_sid: str, _environ: dict, auth: dict | None = None):
    token = None
    if auth:
        token = auth["token"]
    try:
        user = await authenticator.verify_token(token)
        return user
    except AuthenticationError as e:
        default_logger.info(f"authentication failed: {e}")
        return None


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

    default_logger.info("shutdown app")


@contextlib.asynccontextmanager
async def lifespan(_app: FastIO):
    stack = contextlib.AsyncExitStack()
    loop = asyncio.get_event_loop()

    await stack.enter_async_context(get_rmf_events)
    await stack.enter_async_context(get_task_events)
    await stack.enter_async_context(get_fleet_events)
    await stack.enter_async_context(get_alert_events)
    await stack.enter_async_context(get_beacon_events)
    await stack.enter_async_context(get_rio_events)
    await stack.enter_async_context(get_alert_events)

    await Tortoise.init(
        db_url=app_config.db_url,
        modules={"models": ["api_server.models.tortoise_models"]},
    )
    # FIXME: do this outside the app as recommended by the docs
    await Tortoise.generate_schemas()
    shutdown_cbs.append(Tortoise.close_connections())

    await stack.enter_async_context(get_cached_file_repo)
    await stack.enter_async_context(ros.get_ros_node)
    await stack.enter_async_context(gateway.get_rmf_gateway)
    await stack.enter_async_context(get_tasks_service)

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

    await _load_states(get_rmf_events())

    default_logger.info("starting scheduler")
    await stack.enter_async_context(get_scheduler)
    scheduled_tasks = await ttm.ScheduledTask.all()
    scheduled = 0
    for t in scheduled_tasks:
        user = await User.load_from_db(t.created_by)
        if user is None:
            default_logger.warning(f"user [{t.created_by}] does not exist")
            continue
        task_repo = TaskRepository(user, default_logger)
        await routes.scheduled_tasks.schedule_task(
            t,
            task_repo,
            get_tasks_service(),
            get_scheduler(),
            default_logger,
        )
        scheduled += 1
    default_logger.info(f"loaded {scheduled} tasks")
    default_logger.info("successfully started scheduler")

    default_logger.info("started app")

    yield

    await shutdown()
    await stack.aclose()


app = FastIO(
    title="RMF API Server",
    lifespan=lifespan,
    socketio_connect=on_sio_connect,
    docs_url=None,
    redoc_url=None,
)
if app.swagger_ui_oauth2_redirect_url is None:
    app.swagger_ui_oauth2_redirect_url = "docs/oauth2-redirect"


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
    routes.delivery_alerts_router,
    prefix="/delivery_alerts",
    dependencies=[Depends(user_dep)],
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
app.include_router(routes.rios_router, prefix="/rios", dependencies=[Depends(user_dep)])
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


async def _load_states(rmf_events: RmfEvents):
    default_logger.info("loading states from database...")

    door_states = [DoorState.model_validate(x) for x in await ttm.DoorState.all()]
    for state in door_states:
        rmf_events.door_states.on_next(state)
    default_logger.info(f"loaded {len(door_states)} door states")

    lift_states = [LiftState.from_tortoise(x) for x in await ttm.LiftState.all()]
    for state in lift_states:
        rmf_events.lift_states.on_next(state)
    default_logger.info(f"loaded {len(lift_states)} lift states")

    dispenser_states = [
        DispenserState.model_validate(x) for x in await ttm.DispenserState.all()
    ]
    for state in dispenser_states:
        rmf_events.dispenser_states.on_next(state)
    default_logger.info(f"loaded {len(dispenser_states)} dispenser states")

    ingestor_states = [
        IngestorState.model_validate(x) for x in await ttm.IngestorState.all()
    ]
    for state in ingestor_states:
        rmf_events.ingestor_states.on_next(state)
    default_logger.info(f"loaded {len(ingestor_states)} ingestor states")
