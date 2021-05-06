# fastapi relies on global variables
# pylint: disable=global-statement

import asyncio
import os
import threading
from typing import Optional

import rclpy
import rclpy.node
from fastapi import HTTPException
from rmf_door_msgs.msg import DoorRequest
from rmf_lift_msgs.msg import LiftRequest
from rmf_task_msgs.srv import CancelTask, GetTaskList, SubmitTask

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
        self.submit_task_srv = self.create_client(SubmitTask, "submit_task")
        self.get_tasks_srv = self.create_client(GetTaskList, "get_tasks")
        self.cancel_task_srv = self.create_client(CancelTask, "cancel_task")


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
    use_sim_time_env = os.environ.get("RMF_SERVER_USE_SIM_TIME", None)
    if use_sim_time_env:
        use_sim_time = not use_sim_time_env.lower() in ["0", "false"]
    else:
        use_sim_time = False
    if use_sim_time:
        rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    else:
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


async def call_service(client: rclpy.client.Client, req, timeout=1):
    """
    Raises HTTPException if service call fails
    """
    fut = client.call_async(req)
    try:
        result = await asyncio.wait_for(fut, timeout=timeout)
        return result
    except asyncio.TimeoutError as e:
        raise HTTPException(503, "ros service call timed out") from e
