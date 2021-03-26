import logging
import os
import sys
import threading

import rclpy
import socketio
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from tortoise import Tortoise

from . import routes
from .app_config import app_config
from .authenticator import JwtAuthenticator, StubAuthenticator
from .models import tortoise_models as ttm
from .node import RosNode
from .repositories import StaticFilesRepository
from .rmf_io import HealthWatchdog, RmfBookKeeper, RmfGateway, RmfIO, RmfTransport

logger = logging.getLogger("app")
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
logger.setLevel(app_config.log_level)
if "RMF_API_SEVER_LOG_LEVEL" in os.environ:
    logger.setLevel(os.environ["RMF_API_SEVER_LOG_LEVEL"])

app = FastAPI(
    openapi_url=f"{app_config.root_path}/openapi.json",
    docs_url=f"{app_config.root_path}/docs",
    swagger_ui_oauth2_redirect_url=f"{app_config.root_path}/docs/oauth2-redirect",
)
app.mount(
    f"{app_config.static_path}",
    StaticFiles(directory=app_config.static_directory),
    name="static",
)
sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*", logger=logger)
sio_app = socketio.ASGIApp(
    sio, other_asgi_app=app, socketio_path=app_config.socket_io_path
)

rclpy.init()
ros_node = RosNode()

if app_config.jwt_public_key is None:
    auth = StubAuthenticator()
    logger.warning("authentication is disabled")
else:
    auth = JwtAuthenticator(app_config.jwt_public_key)


def ros_main():
    logger.info("start spinning rclpy node")
    rclpy.spin(ros_node)
    logger.info("finished spinning rclpy node")


async def load_doors(gateway: RmfGateway):
    door_states = await ttm.DoorState.all()
    for state in door_states:
        gateway.door_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(door_states)} door states")

    healths = await ttm.DoorHealth.all()
    for health in healths:
        gateway.door_health.on_next(health)
    logger.info(f"loaded {len(healths)} door health")


async def load_lifts(gateway: RmfGateway):
    lift_states = await ttm.LiftState.all()
    for state in lift_states:
        gateway.lift_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(lift_states)} lift states")

    healths = await ttm.LiftHealth.all()
    for health in healths:
        gateway.lift_health.on_next(health)
    logger.info(f"loaded {len(healths)} lift health")


async def load_dispensers(gateway: RmfGateway):
    dispenser_states = await ttm.DispenserState.all()
    for state in dispenser_states:
        gateway.dispenser_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(dispenser_states)} dispenser states")

    healths = await ttm.DispenserHealth.all()
    for health in healths:
        gateway.dispenser_health.on_next(health)
    logger.info(f"loaded {len(healths)} dispenser health")


async def load_ingestors(gateway: RmfGateway):
    ingestor_states = await ttm.IngestorState.all()
    for state in ingestor_states:
        gateway.ingestor_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(ingestor_states)} ingestor states")

    healths = await ttm.IngestorHealth.all()
    for health in healths:
        gateway.ingestor_health.on_next(health)
    logger.info(f"loaded {len(healths)} ingestor health")


async def load_fleets(gateway: RmfGateway):
    fleet_states = await ttm.FleetState.all()
    for state in fleet_states:
        gateway.fleet_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(fleet_states)} fleet states")

    healths = await ttm.RobotHealth.all()
    for health in healths:
        gateway.robot_health.on_next(health)
    logger.info(f"loaded {len(healths)} robot health")


async def load_tasks(gateway: RmfGateway):
    task_summaries = await ttm.TaskSummary.all()
    for task in task_summaries:
        gateway.task_summaries.on_next(task.to_rmf())
    logger.info(f"loaded {len(task_summaries)} tasks")


async def load_states(gateway: RmfGateway):
    logger.info("loading states from database...")

    await load_doors(gateway)
    await load_lifts(gateway)
    await load_dispensers(gateway)
    await load_fleets(gateway)
    await load_tasks(gateway)

    logger.info("successfully loaded all states")


@app.on_event("startup")
async def on_startup():
    await Tortoise.init(
        db_url=app_config.db_url,
        modules={"models": ["api_server.models.tortoise_models"]},
    )
    await Tortoise.generate_schemas()

    os.makedirs(app_config.static_directory, exist_ok=True)
    static_files_repo = StaticFilesRepository(
        app_config.static_path,
        app_config.static_directory,
        logger.getChild("static_files"),
    )

    rmf_gateway = RmfGateway()
    RmfIO(
        sio,
        rmf_gateway,
        static_files_repo,
        logger=logger.getChild("RmfIO"),
        authenticator=auth,
    )

    # loading states involves emitting events to observables in RmfGateway, we need to load states
    # after initializing RmfIO so that new clients continues to receive the same data. BUT we want
    # to load states before initializing some components like the watchdog because we don't want
    # these fake events to affect them. e.g. The fake events from loading states will trigger the
    # health watchdog to think that a dead component has come back alive.
    await load_states(rmf_gateway)

    HealthWatchdog(rmf_gateway, logger=logger.getChild("HealthWatchdog"))

    rmf_transport = RmfTransport(ros_node, rmf_gateway)
    rmf_transport.subscribe_all()

    rmf_bookkeeper = RmfBookKeeper(rmf_gateway, logger=logger.getChild("BookKeeper"))
    rmf_bookkeeper.start()

    app.include_router(
        routes.building_map_router(rmf_gateway),
        prefix=f"{app_config.root_path}/building_map",
    )
    app.include_router(
        routes.doors_router(ros_node), prefix=f"{app_config.root_path}/doors"
    )
    app.include_router(
        routes.lifts_router(ros_node), prefix=f"{app_config.root_path}/lifts"
    )

    threading.Thread(target=ros_main).start()

    logger.info("started app")


@app.on_event("shutdown")
async def on_shutdown():
    rclpy.shutdown()
    await Tortoise.close_connections()
    logger.info("shutdown app")
