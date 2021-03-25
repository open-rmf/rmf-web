import argparse
import json
import logging
import math
import os
import sys
import time
from threading import Thread

import rclpy
import yaml
from fastapi import APIRouter
from pydantic import BaseModel
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, qos_profile_system_default
from rclpy.time import Time
from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.msg import Delivery, Loop, TaskType
from rmf_task_msgs.srv import SubmitTask

# from typing import Optional


router = APIRouter(prefix="/tasks", tags=["tasks"])

# response type
class Description(BaseModel):
    start_name: str
    finish_name: str
    num_loops: int


class Submit_Task(BaseModel):
    task_type: str
    start_time: int
    description: Description


# dispatcher class
class DispatcherClient(Node):
    def __init__(self):
        super().__init__("dispatcher_client")
        self.submit_task_srv = self.create_client(SubmitTask, "/submit_task")
        counter = 10

        print("starting up....... the dispatcher")
        print(self.submit_task_srv.wait_for_service(timeout_sec=1.0))

        while (
            not self.submit_task_srv.wait_for_service(timeout_sec=1.0) and counter > 0
        ):
            self.get_logger().warn("Dispatcher node is not avail, waiting...")
            counter -= 1

        print(self.submit_task_srv.wait_for_service(timeout_sec=1.0))


rclpy.init(args=None)
dispatcher_client = DispatcherClient()


@router.post("/submit_task")
async def submit_task(submit_task_params: Submit_Task):
    return submit_task_params
