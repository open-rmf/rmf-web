from typing import cast

from builtin_interfaces.msg import Time as RosTime
from fastapi import HTTPException
from rmf_task_msgs.msg import Delivery as RmfDelivery
from rmf_task_msgs.msg import Loop as RmfLoop
from rmf_task_msgs.msg import TaskType as RmfTaskType
from rmf_task_msgs.srv import SubmitTask as RmfSubmitTask

from api_server.models import (
    CleanTaskDescription,
    DeliveryTaskDescription,
    LoopTaskDescription,
    SubmitTask,
    TaskTypeEnum,
)
from api_server.models import tortoise_models as ttm
from api_server.ros_time import convert_to_rmf_time


def convert_task_request(task_request: SubmitTask, now: RosTime):
    """
    :param (obj) task_json:
    :return req_msgs, error_msg
    This is to convert a json task req format to a rmf_task_msgs
    task_profile format. add this accordingly when a new msg field
    is introduced.
    The 'start time' here is refered to the "Duration" from now.
    """

    # NOTE: task request should already be validated by pydantic
    req_msg = RmfSubmitTask.Request()
    req_msg.requester = "rmf_server"  # TODO: Set this as the user id
    if task_request.priority is not None:
        req_msg.description.priority.value = task_request.priority

    if task_request.task_type == TaskTypeEnum.CLEAN:
        clean_desc = cast(CleanTaskDescription, task_request.description)
        req_msg.description.task_type.type = RmfTaskType.TYPE_CLEAN
        req_msg.description.clean.start_waypoint = clean_desc.cleaning_zone
    elif task_request.task_type == TaskTypeEnum.LOOP:
        loop_desc = cast(LoopTaskDescription, task_request.description)
        req_msg.description.task_type.type = RmfTaskType.TYPE_LOOP
        loop = RmfLoop()
        loop.num_loops = loop_desc.num_loops
        loop.start_name = loop_desc.start_name
        loop.finish_name = loop_desc.finish_name
        req_msg.description.loop = loop
    elif task_request.task_type == TaskTypeEnum.DELIVERY:
        delivery_desc = cast(DeliveryTaskDescription, task_request.description)
        req_msg.description.task_type.type = RmfTaskType.TYPE_DELIVERY
        delivery = RmfDelivery()
        delivery.pickup_place_name = delivery_desc.pickup_place_name
        delivery.pickup_dispenser = delivery_desc.pickup_dispenser
        delivery.dropoff_ingestor = delivery_desc.dropoff_ingestor
        delivery.dropoff_place_name = delivery_desc.dropoff_place_name
        req_msg.description.delivery = delivery
    else:
        return None, "Invalid TaskType"

    rmf_start_time = convert_to_rmf_time(task_request.start_time, now)
    req_msg.description.start_time = rmf_start_time
    return req_msg, ""


async def get_db_task(task_id: str) -> ttm.TaskSummary:
    task = await ttm.TaskSummary.get_or_none(id_=task_id)
    if task is None:
        raise HTTPException(404)
    return task
