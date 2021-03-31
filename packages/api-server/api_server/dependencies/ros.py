# fastapi relies on global variables
# pylint: disable=global-statement

import threading
from typing import Optional

import rclpy
import rclpy.node
from rmf_door_msgs.msg import DoorRequest
from rmf_lift_msgs.msg import LiftRequest

from ..rmf_io import RmfGateway
from .logging import logger as parent_logger

logger = parent_logger.getChild("ros")
logger.parent = parent_logger

rmf_gateway = RmfGateway()


class RosNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("rmf_api_server")
        self.door_req = self.create_publisher(DoorRequest, "adapter_door_requests", 10)
        self.lift_req = self.create_publisher(LiftRequest, "adapter_lift_requests", 10)


node: Optional[RosNode] = None


def _spin():
    logger.info("start spinning rclpy node")
    rclpy.spin(node)
    logger.info("finished spinning rclpy node")


spin_thread: Optional[threading.Thread] = None


def on_startup():
    """
    Must be called on app startup
    """
    rclpy.init()
    global node
    node = RosNode()


def start_spin():
    """
    Must be called on app startup, AFTER subscriptions have been created
    """
    global spin_thread
    spin_thread = threading.Thread(target=_spin)
    spin_thread.start()


def on_shutdown():
    """
    Must be called on app shutdown
    """
    rclpy.shutdown()
    logger.info("waiting for rclpy spin thread to exit")
    global spin_thread
    spin_thread.join()
    spin_thread = None
    global node
    node = None
