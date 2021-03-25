import logging
import os
import sys
import threading

import rclpy
import socketio
from rclpy.node import Node
from tortoise import Tortoise

from . import models
from .app_config import app_config
from .repositories import StaticFilesRepository
from .rmf_io import HealthWatchdog, RmfBookKeeper, RmfGateway, RmfIO, RmfTransport


class MainNode(Node):
    def __init__(self):
        super().__init__("rmf_api_server")


def ros2_thread(node):
    print("entering ros2 thread")
    rclpy.spin(node)
    print("leaving ros2 thread")


async def init_tortoise():
    await Tortoise.init(
        db_url=app_config.db_url, modules={"models": ["api_server.models"]}
    )
    await Tortoise.generate_schemas()


logger = logging.getLogger("app")
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
logger.setLevel(app_config.log_level)

sio_client = socketio.AsyncClient()


async def load_doors(gateway: RmfGateway):
    door_states = await models.DoorState.all()
    for state in door_states:
        gateway.door_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(door_states)} door states")

    healths = await models.DoorHealth.all()
    for health in healths:
        gateway.door_health.on_next(health)
    logger.info(f"loaded {len(healths)} door health")


async def load_lifts(gateway: RmfGateway):
    lift_states = await models.LiftState.all()
    for state in lift_states:
        gateway.lift_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(lift_states)} lift states")

    healths = await models.LiftHealth.all()
    for health in healths:
        gateway.lift_health.on_next(health)
    logger.info(f"loaded {len(healths)} lift health")


async def load_dispensers(gateway: RmfGateway):
    dispenser_states = await models.DispenserState.all()
    for state in dispenser_states:
        gateway.dispenser_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(dispenser_states)} dispenser states")

    healths = await models.DispenserHealth.all()
    for health in healths:
        gateway.dispenser_health.on_next(health)
    logger.info(f"loaded {len(healths)} dispenser health")


async def load_ingestors(gateway: RmfGateway):
    ingestor_states = await models.IngestorState.all()
    for state in ingestor_states:
        gateway.ingestor_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(ingestor_states)} ingestor states")

    healths = await models.IngestorHealth.all()
    for health in healths:
        gateway.ingestor_health.on_next(health)
    logger.info(f"loaded {len(healths)} ingestor health")


async def load_fleets(gateway: RmfGateway):
    fleet_states = await models.FleetState.all()
    for state in fleet_states:
        gateway.fleet_states.on_next(state.to_rmf())
    logger.info(f"loaded {len(fleet_states)} fleet states")

    healths = await models.RobotHealth.all()
    for health in healths:
        gateway.robot_health.on_next(health)
    logger.info(f"loaded {len(healths)} robot health")


async def load_tasks(gateway: RmfGateway):
    task_summaries = await models.TaskSummary.all()
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


async def on_startup():
    await init_tortoise()

    rclpy.init(args=None)
    ros2_node = MainNode()

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
    )

    # loading states involves emitting events to observables in RmfGateway, we need to load states
    # after initializing RmfIO so that new clients continues to receive the same data. BUT we want
    # to load states before initializing some components like the watchdog because we don't want
    # these fake events to affect them. e.g. The fake events from loading states will trigger the
    # health watchdog to think that a dead component has come back alive.
    await load_states(rmf_gateway)

    HealthWatchdog(rmf_gateway, logger=logger.getChild("HealthWatchdog"))

    rmf_transport = RmfTransport(ros2_node, rmf_gateway)
    rmf_transport.subscribe_all()

    rmf_bookkeeper = RmfBookKeeper(rmf_gateway, logger=logger.getChild("BookKeeper"))
    rmf_bookkeeper.start()

    threading.Thread(target=ros2_thread, args=[ros2_node]).start()
    logger.info("started app")


async def on_shutdown():
    rclpy.shutdown()
    await Tortoise.close_connections()
    logger.info("shutdown app")


# TODO - change cors_allowed_origin to more specific origin. Putting a wild card for now
sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")
app = socketio.ASGIApp(
    sio,
    static_files={app_config.static_path: app_config.static_directory},
    on_startup=on_startup,
    on_shutdown=on_shutdown,
)
