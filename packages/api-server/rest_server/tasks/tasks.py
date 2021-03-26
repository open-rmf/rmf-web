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
from fastapi import APIRouter, Request
from fastapi.encoders import jsonable_encoder
from fastapi.responses import JSONResponse
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

    def submit_task_request(self, req_msg) -> str:
        """
        Task Submission - This function will trigger a ros srv call to the
        dispatcher node, and return a response. Function will return a Task ID
        """
        try:
            future = self.submit_task_srv.call_async(req_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            response = future.result()
            if response is None:
                self.get_logger().warn("/submit_task srv call failed")
            elif not response.success:
                self.node.get_logger().error("Dispatcher node failed to accept task")
            else:
                self.get_logger().info(f"New Dispatch task_id {response.task_id}")
                return response.task_id
        except Exception as e:
            self.get_logger().error("Error! Submit Srv failed %r" % (e,))
        return ""

    def convert_task_request(self, task_json):
        """
        :param (obj) task_json:
        :return req_msgs, error_msg
        This is to convert a json task req format to a rmf_task_msgs
        task_profile format. add this accordingly when a new msg field
        is introduced.
        The 'start time' here is refered to the "Duration" from now.
        """
        req_msg = SubmitTask.Request()

        try:
            if (
                ("task_type" not in task_json)
                or ("start_time" not in task_json)
                or ("description" not in task_json)
            ):
                raise Exception("Key value is incomplete")

            if "priority" in task_json:
                priority = int(task_json["priority"])
                if priority < 0:
                    raise Exception("Priority value is less than 0")
                req_msg.description.priority.value = priority
            else:
                req_msg.description.priority.value = 0

            desc = task_json["description"]
            if task_json["task_type"] == "Clean":
                req_msg.description.task_type.type = TaskType.TYPE_CLEAN
                req_msg.description.clean.start_waypoint = desc["cleaning_zone"]
            elif task_json["task_type"] == "Loop":
                req_msg.description.task_type.type = TaskType.TYPE_LOOP
                loop = Loop()
                loop.num_loops = int(desc["num_loops"])
                loop.start_name = desc["start_name"]
                loop.finish_name = desc["finish_name"]
                req_msg.description.loop = loop
            elif task_json["task_type"] == "Delivery":
                req_msg.description.task_type.type = TaskType.TYPE_DELIVERY
                delivery = Delivery()
                delivery.pickup_place_name = desc["pickup_place_name"]
                delivery.pickup_dispenser = desc["pickup_dispenser"]
                delivery.dropoff_ingestor = desc["dropoff_ingestor"]
                delivery.dropoff_place_name = desc["dropoff_place_name"]
                req_msg.description.delivery = delivery
            else:
                raise Exception("Invalid TaskType")

            # Calc start time, convert min to sec: TODO better representation
            rclpy.spin_once(self, timeout_sec=0.0)
            ros_start_time = self.get_clock().now().to_msg()
            ros_start_time.sec += int(task_json["start_time"] * 60)
            req_msg.description.start_time = ros_start_time
        except KeyError as ex:
            return None, f"Missing Key value in json body: {ex}"
        except Exception as ex:
            return None, str(ex)
        return req_msg, ""


rclpy.init(args=None)
dispatcher_client = DispatcherClient()


@router.post("/submit_task")
async def submit_task(submit_task_params: Request):
    params_to_dict = await submit_task_params.json()
    req_msg, err_msg = dispatcher_client.convert_task_request(params_to_dict)

    if req_msg:
        task_id = dispatcher_client.submit_task_request(req_msg)
        if task_id:
            return JSONResponse(content={"task_id": task_id, "error_msg": ""})

    return JSONResponse(content={"error_msg": err_msg})
