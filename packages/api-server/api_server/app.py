import asyncio
import importlib
import logging
import os
import sys
import threading

import rclpy
from rclpy.node import Node

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from .app_config import AppConfig
from .building_map import building_map_router
from .repositories.static_files import StaticFilesRepository
from .rmf_io import RmfIO


class MainNode(Node):
    def __init__(self):
        super().__init__('rmf_api_server')


def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')


def load_config(config_file: str) -> AppConfig:
    spec = importlib.util.spec_from_file_location('config', config_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return AppConfig(module.config)


if 'RMF_API_SERVER_CONFIG' in os.environ:
    config_file = os.environ['RMF_API_SERVER_CONFIG']
else:
    config_file = f'{os.path.dirname(__file__)}/default_config.py'
app_config = load_config(config_file)

logger = logging.getLogger('app')
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
logger.setLevel(logging.INFO)

app = FastAPI()
os.makedirs(app_config.static_directory, exist_ok=True)
static_files = StaticFiles(directory=app_config.static_directory)
app.mount(app_config.static_path, static_files, name='static')
static_files_repo = StaticFilesRepository(
    app_config.static_path, app_config.static_directory)
ros2_node: Node


@app.on_event('startup')
def start_rclpy():
    global ros2_node
    rclpy.init(args=None)
    ros2_node = MainNode()
    threading.Thread(target=ros2_thread, args=[ros2_node]).start()

    rmfio = RmfIO(asyncio.get_running_loop(), logger.getChild('RmfIO'))
    # app.include_router(
    #     building_map_router(ros2_node, static_files_repo,
    #                         logger.getChild('building_map')),
    #     prefix='/building_map')


@app.on_event('shutdown')
def shutdown_rclpy():
    rclpy.shutdown()
