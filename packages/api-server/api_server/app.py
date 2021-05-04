import asyncio
import os
import signal
from typing import Awaitable, Callable, List, Union

import socketio
from fastapi import Depends, FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from tortoise import Tortoise

from . import routes
from .app_config import app_config
from .dependencies import auth_scheme, authenticator, logger, ros
from .models import tortoise_models as ttm
from .repositories import StaticFilesRepository
from .rmf_io import HealthWatchdog, RmfBookKeeper, RmfIO, RmfTransport

# will be called in reverse order on app shutdown
shutdown_cbs: List[Callable[[], Union[None, Awaitable[None]]]] = []


if "CI" in os.environ:
    app_config.iss = "https://example.com/auth/realms/rmf-web"

app = FastAPI(
    dependencies=[Depends(auth_scheme)],
)
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
sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*", logger=logger)
sio_app = socketio.ASGIApp(sio, other_asgi_app=app)

app.include_router(
    routes.building_map_router,
    prefix="/building_map",
)
app.include_router(routes.doors_router, prefix="/doors")
app.include_router(routes.lifts_router, prefix="/lifts")
app.include_router(routes.tasks_router, prefix="/tasks")


async def load_doors():
    door_states = await ttm.DoorState.all()
    for state in door_states:
        ros.rmf_gateway.door_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(door_states)} door states")

    healths = await ttm.DoorHealth.all()
    for health in healths:
        ros.rmf_gateway.door_health.on_next(health)
    logger.info(f"loaded {len(healths)} door health")


async def load_lifts():
    lift_states = await ttm.LiftState.all()
    for state in lift_states:
        ros.rmf_gateway.lift_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(lift_states)} lift states")

    healths = await ttm.LiftHealth.all()
    for health in healths:
        ros.rmf_gateway.lift_health.on_next(health)
    logger.info(f"loaded {len(healths)} lift health")


async def load_dispensers():
    dispenser_states = await ttm.DispenserState.all()
    for state in dispenser_states:
        ros.rmf_gateway.dispenser_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(dispenser_states)} dispenser states")

    healths = await ttm.DispenserHealth.all()
    for health in healths:
        ros.rmf_gateway.dispenser_health.on_next(health)
    logger.info(f"loaded {len(healths)} dispenser health")


async def load_ingestors():
    ingestor_states = await ttm.IngestorState.all()
    for state in ingestor_states:
        ros.rmf_gateway.ingestor_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(ingestor_states)} ingestor states")

    healths = await ttm.IngestorHealth.all()
    for health in healths:
        ros.rmf_gateway.ingestor_health.on_next(health)
    logger.info(f"loaded {len(healths)} ingestor health")


async def load_fleets():
    fleet_states = await ttm.FleetState.all()
    for state in fleet_states:
        ros.rmf_gateway.fleet_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(fleet_states)} fleet states")

    healths = await ttm.RobotHealth.all()
    for health in healths:
        ros.rmf_gateway.robot_health.on_next(health)
    logger.info(f"loaded {len(healths)} robot health")


async def load_tasks():
    task_summaries = await ttm.TaskSummary.all()
    for task in task_summaries:
        ros.rmf_gateway.task_summaries.on_next(task.to_rmf())
    logger.info(f"loaded {len(task_summaries)} tasks")


async def load_states():
    logger.info("loading states from database...")

    await load_doors()
    await load_lifts()
    await load_dispensers()
    await load_fleets()
    await load_tasks()

    logger.info("successfully loaded all states")


@app.on_event("startup")
async def on_startup():
    # for some unexplainable reason, when running with "npx concurrently", the signals
    # are not captured and the app is not shutdown. Capturing the signal manually makes
    # it work for some reason.
    loop = asyncio.get_event_loop()

    def on_signal(_sig, _frame):
        loop.create_task(on_shutdown())

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    await Tortoise.init(
        db_url=app_config.db_url,
        modules={"models": ["api_server.models.tortoise_models"]},
    )
    await Tortoise.generate_schemas()
    shutdown_cbs.append(Tortoise.close_connections)

    static_files_repo = StaticFilesRepository(
        f"{app_config.public_url.geturl()}/static",
        app_config.static_directory,
        logger.getChild("static_files"),
    )
    health_watchdog = HealthWatchdog(
        ros.rmf_gateway, logger=logger.getChild("HealthWatchdog")
    )
    rmf_bookkeeper = RmfBookKeeper(
        ros.rmf_gateway, logger=logger.getChild("BookKeeper")
    )
    rmf_io = RmfIO(
        sio,
        ros.rmf_gateway,
        static_files_repo,
        logger=logger.getChild("RmfIO"),
        authenticator=authenticator,
    )
    rmf_io.start()
    shutdown_cbs.append(rmf_io.stop)

    # loading states involves emitting events to observables in RmfGateway, we need to load states
    # after initializing RmfIO so that new clients continues to receive the same data. BUT we want
    # to load states before initializing some components like the watchdog because we don't want
    # these fake events to affect them. e.g. The fake events from loading states will trigger the
    # health watchdog to think that a dead component has come back alive.
    await load_states()

    health_watchdog.start()
    shutdown_cbs.append(health_watchdog.stop)
    rmf_bookkeeper.start()
    shutdown_cbs.append(rmf_bookkeeper.stop)

    ros.on_startup()
    shutdown_cbs.append(ros.on_shutdown)

    rmf_transport = RmfTransport(ros.rmf_gateway)
    rmf_transport.subscribe_all(ros.node)
    shutdown_cbs.append(rmf_transport.unsubscribe_all)

    ros.start_spin()

    logger.info("started app")


@app.on_event("shutdown")
async def on_shutdown():
    while shutdown_cbs:
        cb = shutdown_cbs.pop()
        result = cb()
        if asyncio.iscoroutine(result):
            await result

    logger.info("shutdown app")
