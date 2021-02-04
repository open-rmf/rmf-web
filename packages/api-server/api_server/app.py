import asyncio
import importlib
import logging
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node

import socketio

from tortoise import Tortoise

from .app_config import app_config, AppConfig
from .repositories.static_files import StaticFilesRepository
from .rmf_io import RmfIO, RmfGateway, RmfTransport, RmfBookKeeper


class MainNode(Node):
    def __init__(self):
        super().__init__('rmf_api_server')


def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')


async def init_tortoise(app_config: AppConfig):
    await Tortoise.init(db_url=app_config.db_url, modules={'models': ['api_server.models']})
    await Tortoise.generate_schemas()


sio_client = socketio.AsyncClient()


async def on_startup():
    await init_tortoise(app_config)

    global ros2_node
    rclpy.init(args=None)
    ros2_node = MainNode()

    os.makedirs(app_config.static_directory, exist_ok=True)
    static_files_repo = StaticFilesRepository(
        app_config.static_path, app_config.static_directory, logger.getChild('static_files'))

    rmf_gateway = RmfGateway()
    rmf_io = RmfIO(sio, rmf_gateway, static_files_repo,
                   logger=logger.getChild('RmfIO'))

    rmf_transport = RmfTransport(ros2_node, rmf_gateway)
    rmf_transport.subscribe_all()

    rmf_bookkeeper = RmfBookKeeper(
        rmf_gateway, logger=logger.getChild('BookKeeper'))
    rmf_bookkeeper.start()

    threading.Thread(target=ros2_thread, args=[ros2_node]).start()
    logger.info('started app')


async def on_shutdown():
    rclpy.shutdown()
    await Tortoise.close_connections()
    logger.info('shutdown app')

logger = logging.getLogger('app')
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
if 'RMF_API_SERVER_DEBUG' in os.environ:
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.INFO)
ros2_node: Node

sio = socketio.AsyncServer(async_mode='asgi')
app = socketio.ASGIApp(
    sio,
    static_files={
        app_config.static_path: app_config.static_directory
    },
    on_startup=on_startup,
    on_shutdown=on_shutdown)
