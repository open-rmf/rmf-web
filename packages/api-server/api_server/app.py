import logging
import os
import sys
import threading
from typing import List

import rclpy
import socketio
from rclpy.node import Node
from tortoise import Tortoise

from .app_config import app_config
from .models import DispenserHealth, DoorHealth, LiftHealth
from .repositories import RmfRepository, SqlRepository, StaticFilesRepository
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
if "RMF_API_SERVER_DEBUG" in os.environ:
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.INFO)

sio_client = socketio.AsyncClient()


async def load_states(repo: RmfRepository, gateway: RmfGateway):
    logger.info("loading states from database...")

    door_states = await repo.read_door_states()
    for state in door_states:
        gateway.door_states.on_next(state)
    logger.info(f"loaded {len(door_states)} door states")

    door_health: List[DoorHealth] = []
    for state in door_states.values():
        health = await repo.read_door_health(state.door_name)
        if health:
            door_health.append(health)
    for health in door_health:
        gateway.door_health.on_next(health)
    logger.info(f"loaded {len(door_health)} door health")

    lift_states = await repo.read_lift_states()
    for state in lift_states:
        gateway.lift_states.on_next(state)
    logger.info(f"loaded {len(lift_states)} lift states")

    lift_health: List[LiftHealth] = []
    for state in lift_states.values():
        health = await repo.read_lift_health(state.lift_name)
        if health:
            lift_health.append(health)
    for health in lift_health:
        gateway.lift_health.on_next(health)
    logger.info(f"loaded {len(lift_health)} lift health")

    dispenser_states = await repo.read_dispenser_states()
    for state in dispenser_states:
        gateway.dispenser_states.on_next(state)
    logger.info(f"loaded {len(dispenser_states)} dispenser states")

    dispenser_health: List[DispenserHealth] = []
    for state in dispenser_states.values():
        health = await repo.read_dispenser_health(state.guid)
        if health:
            dispenser_health.append(health)
    for health in dispenser_health:
        gateway.dispenser_health.on_next(health)
    logger.info(f"loaded {len(dispenser_health)} dispenser health")

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

    sql_repo = SqlRepository(logger.getChild("SqlRepository"))
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
    await load_states(sql_repo, rmf_gateway)

    HealthWatchdog(rmf_gateway, logger=logger.getChild("HealthWatchdog"))

    rmf_transport = RmfTransport(ros2_node, rmf_gateway)
    rmf_transport.subscribe_all()

    rmf_bookkeeper = RmfBookKeeper(
        rmf_gateway, sql_repo, logger=logger.getChild("BookKeeper")
    )
    rmf_bookkeeper.start()

    threading.Thread(target=ros2_thread, args=[ros2_node]).start()
    logger.info("started app")


async def on_shutdown():
    rclpy.shutdown()
    await Tortoise.close_connections()
    logger.info("shutdown app")


sio = socketio.AsyncServer(async_mode="asgi")
app = socketio.ASGIApp(
    sio,
    static_files={app_config.static_path: app_config.static_directory},
    on_startup=on_startup,
    on_shutdown=on_shutdown,
)
