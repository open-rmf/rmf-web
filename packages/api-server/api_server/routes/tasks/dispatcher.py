from typing import List, Sequence

from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary
from rmf_task_msgs.srv import CancelTask, GetTaskList, SubmitTask
from rosidl_runtime_py.convert import message_to_ordereddict

from ... import models as mdl
from ...gateway import RmfGateway
from ...models.tasks import TaskProgress


class DispatcherClient:
    def __init__(self, rmf_gateway: RmfGateway):
        self.rmf_gateway = rmf_gateway

    async def submit_task_request(self, req_msg: SubmitTask.Request):
        """
        Task Submission - This function will trigger a ros srv call to the
        dispatcher node, and return a response. Function will return a Task ID.
        Raises "HTTPException" if service call fails.
        """
        resp: SubmitTask.Response = await self.rmf_gateway.call_service(
            self.rmf_gateway.submit_task_srv, req_msg
        )
        return resp

    def get_sim_time(self):
        if self.rmf_gateway.get_parameter("use_sim_time").value:
            sim_time = self.rmf_gateway.get_clock().now().to_msg()
        else:
            sim_time = None
        return sim_time

    async def get_task_status(self) -> List[TaskProgress]:
        """
        Get all task status - This fn will trigger a ros srv call to acquire
        all submitted tasks to dispatcher node. Fn returns an object of tasks.
        Raises "HTTPException" if service call fails.
        """
        req = GetTaskList.Request()
        response = await self.rmf_gateway.call_service(
            self.rmf_gateway.get_tasks_srv, req
        )
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
        response = await self.rmf_gateway.call_service(
            self.rmf_gateway.cancel_task_srv, req
        )
        return response.success

    def __convert_task_status_msg(self, task_summaries: Sequence[RmfTaskSummary]):
        """
        convert rmf task summary msg to a task
        """
        tasks = []
        now = self.rmf_gateway.get_clock().now().to_msg().sec  # only use sec
        for rmf_task in task_summaries:
            # Generate a progress percentage
            duration = abs(rmf_task.end_time.sec - rmf_task.start_time.sec)
            # check if is completed
            if rmf_task.state == RmfTaskSummary.STATE_COMPLETED:
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
                TaskProgress(
                    task_summary=message_to_ordereddict(rmf_task), progress=progress
                )
            )
        return tasks
