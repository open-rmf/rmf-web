from typing import List, Sequence, cast

from rmf_task_msgs.msg import Delivery, Loop, TaskSummary, TaskType
from rmf_task_msgs.srv import CancelTask, GetTaskList, SubmitTask
from rosidl_runtime_py.convert import message_to_ordereddict

from ... import models as mdl
from ...dependencies import ros
from ...models.tasks import Task
from ...ros_time import convert_to_rmf_time


# dispatcher class
class DispatcherClient:
    async def submit_task_request(self, req_msg: SubmitTask.Request):
        """
        Task Submission - This function will trigger a ros srv call to the
        dispatcher node, and return a response. Function will return a Task ID.
        Raises "HTTPException" if service call fails.
        """
        resp: SubmitTask.Response = await ros.call_service(
            ros.node.submit_task_srv, req_msg
        )
        return resp

    @staticmethod
    def convert_task_request(task_request: mdl.SubmitTask):
        """
        :param (obj) task_json:
        :return req_msgs, error_msg
        This is to convert a json task req format to a rmf_task_msgs
        task_profile format. add this accordingly when a new msg field
        is introduced.
        The 'start time' here is refered to the "Duration" from now.
        """

        # NOTE: task request should already be validated by pydantic

        req_msg = SubmitTask.Request()
        req_msg.requester = "rmf_server"  # TODO: Set this as the user id
        if task_request.priority is not None:
            req_msg.description.priority.value = task_request.priority

        if task_request.task_type == mdl.TaskTypeEnum.CLEAN:
            desc = cast(mdl.CleanTaskDescription, task_request.description)
            req_msg.description.task_type.type = TaskType.TYPE_CLEAN
            req_msg.description.clean.start_waypoint = desc.cleaning_zone
        elif task_request.task_type == mdl.TaskTypeEnum.LOOP:
            desc = cast(mdl.LoopTaskDescription, task_request.description)
            req_msg.description.task_type.type = TaskType.TYPE_LOOP
            loop = Loop()
            loop.num_loops = desc.num_loops
            loop.start_name = desc.start_name
            loop.finish_name = desc.finish_name
            req_msg.description.loop = loop
        elif task_request.task_type == mdl.TaskTypeEnum.DELIVERY:
            desc = cast(mdl.DeliveryTaskDescription, task_request.description)
            req_msg.description.task_type.type = TaskType.TYPE_DELIVERY
            delivery = Delivery()
            delivery.pickup_place_name = desc.pickup_place_name
            delivery.pickup_dispenser = desc.pickup_dispenser
            delivery.dropoff_ingestor = desc.dropoff_ingestor
            delivery.dropoff_place_name = desc.dropoff_place_name
            req_msg.description.delivery = delivery
        else:
            return None, "Invalid TaskType"

        rmf_start_time = convert_to_rmf_time(task_request.start_time, ros.node)
        req_msg.description.start_time = rmf_start_time
        return req_msg, ""

    async def get_task_status(self) -> List[Task]:
        """
        Get all task status - This fn will trigger a ros srv call to acquire
        all submitted tasks to dispatcher node. Fn returns an object of tasks.
        Raises "HTTPException" if service call fails.
        """
        req = GetTaskList.Request()
        response = await ros.call_service(ros.node.get_tasks_srv, req)
        return [
            *self.__convert_task_status_msg(response.active_tasks),
            *self.__convert_task_status_msg(response.terminated_tasks),
        ]

    async def cancel_task_request(self, task: mdl.CancelTask) -> bool:
        """
        Cancel Task - This function will trigger a ros srv call to the
        dispatcher node, and return a response.
        Raises "HTTPException" if service call fails.
        """
        req = CancelTask.Request()
        req.task_id = task.task_id
        response = await ros.call_service(ros.node.cancel_task_srv, req)
        return response.success

    @staticmethod
    def __convert_task_status_msg(task_summaries: Sequence[TaskSummary]):
        """
        convert rmf task summary msg to a task
        """
        tasks = []
        now = ros.node.get_clock().now().to_msg().sec  # only use sec
        for rmf_task in task_summaries:
            # Generate a progress percentage
            duration = abs(rmf_task.end_time.sec - rmf_task.start_time.sec)
            # check if is completed
            if rmf_task.state == TaskSummary.STATE_COMPLETED:
                progress = "100%"
            # check if it state is queued/cancelled
            elif duration == 0 or (rmf_task.state in [0, 4]):
                progress = "0%"
            else:
                percent = int(100 * (now - rmf_task.start_time.sec) / float(duration))
                if percent < 0:
                    progress = "0%"
                elif percent > 100:
                    progress = "Delayed"
                else:
                    progress = f"{percent}%"
            tasks.append(
                Task(task_summary=message_to_ordereddict(rmf_task), progress=progress)
            )
        return tasks


task_dispatcher = DispatcherClient()
