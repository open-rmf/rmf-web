from typing import cast

from builtin_interfaces.msg import Time as RosTime
from fastapi.encoders import jsonable_encoder
from rmf_task_msgs.msg import Delivery, Loop, TaskSummary, TaskType
from rmf_task_msgs.srv import CancelTask, GetTaskList, SubmitTask

from ... import models as mdl
from ...dependencies import ros


class StatusModel:
    task_id: str
    state: str
    done: bool
    fleet_name: str
    robot_name: str
    task_type: str
    priority: int
    submited_start_time: int
    start_time: int
    end_time: int
    description: str
    progress: str


# dispatcher class
class DispatcherClient:
    async def submit_task_request(self, req_msg: SubmitTask.Request) -> str:
        """
        Task Submission - This function will trigger a ros srv call to the
        dispatcher node, and return a response. Function will return a Task ID.
        Raises "HTTPException" if service call fails.
        """
        return await ros.call_service(ros.node.submit_task_srv, req_msg)

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

        # Calc start time, convert min to sec: TODO better representation
        ros_start_time = RosTime()
        ros_start_time.sec = task_request.start_time
        req_msg.description.start_time = ros_start_time
        return req_msg, ""

    async def get_task_status(self):
        """
        Get all task status - This fn will trigger a ros srv call to acquire
        all submitted tasks to dispatcher node. Fn returns an object of tasks.
        Raises "HTTPException" if service call fails.
        """
        req = GetTaskList.Request()
        response = await ros.call_service(ros.node.get_tasks_srv, req)
        active_tasks = self.__convert_task_status_msg(response.active_tasks, False)
        terminated_tasks = self.__convert_task_status_msg(
            response.terminated_tasks, True
        )
        return {
            "active_tasks": active_tasks,
            "terminated_tasks": terminated_tasks,
        }

    async def cancel_task_request(self, task_id: mdl.TaskId) -> bool:
        """
        Cancel Task - This function will trigger a ros srv call to the
        dispatcher node, and return a response.
        Raises "HTTPException" if service call fails.
        """
        req = CancelTask.Request()
        req.task_id = task_id.task_id
        response = await ros.call_service(ros.node.cancel_task_srv, req)
        return response.success

    @staticmethod
    def __convert_task_status_msg(task_summaries, is_done=True):
        """
        convert task summary msg and return a jsonify-able task status obj
        """
        states_enum = {
            TaskSummary.STATE_QUEUED: "Queued",
            TaskSummary.STATE_ACTIVE: "Active/Executing",
            TaskSummary.STATE_COMPLETED: "Completed",
            TaskSummary.STATE_FAILED: "Failed",
            TaskSummary.STATE_CANCELED: "Cancelled",
            TaskSummary.STATE_PENDING: "Pending",
        }
        type_enum = {
            TaskType.TYPE_STATION: "Station",
            TaskType.TYPE_LOOP: "Loop",
            TaskType.TYPE_DELIVERY: "Delivery",
            TaskType.TYPE_CHARGE_BATTERY: "Charging",
            TaskType.TYPE_CLEAN: "Clean",
            TaskType.TYPE_PATROL: "Patrol",
        }

        status_list = []
        now = ros.node.get_clock().now().to_msg().sec  # only use sec
        for task in task_summaries:
            desc = task.task_profile.description
            status = StatusModel()
            status.task_id = task.task_id
            status.state = states_enum[task.state]
            status.fleet_name = task.fleet_name
            status.robot_name = task.robot_name
            status.task_type = type_enum[desc.task_type.type]
            status.priority = desc.priority.value
            status.submited_start_time = desc.start_time.sec
            status.start_time = task.start_time.sec  # only use sec
            status.end_time = task.end_time.sec  # only use sec

            if status.task_type == "Clean":
                status.description = desc.clean.start_waypoint
            elif status.task_type == "Loop":
                status.description = (
                    desc.loop.start_name
                    + " --> "
                    + desc.loop.finish_name
                    + " x"
                    + str(desc.loop.num_loops)
                )
            elif status.task_type == "Delivery":
                status.description = (
                    desc.delivery.pickup_place_name
                    + " --> "
                    + desc.delivery.dropoff_place_name
                )
            elif status.task_type == "Charging":
                status.description = "Back to Charging Station"
            else:
                status.description = "Unknown Task Type"

            # Generate a progress percentage
            duration = abs(task.end_time.sec - task.start_time.sec)
            # check if is completed
            if is_done or task.state == TaskSummary.STATE_FAILED:
                status.progress = "100%"
            # check if it state is queued/cancelled
            elif duration == 0 or (task.state in [0, 4]):
                status.progress = "0%"
            else:
                percent = int(100 * (now - task.start_time.sec) / float(duration))
                if percent < 0:
                    status.progress = "0%"
                elif percent > 100:
                    status.progress = "Delayed"
                else:
                    status.progress = f"{percent}%"
            status_list.insert(0, jsonable_encoder(status))
        return status_list


task_dispatcher = DispatcherClient()
