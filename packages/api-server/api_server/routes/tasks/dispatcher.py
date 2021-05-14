from typing import List, Optional

from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary
from rmf_task_msgs.srv import CancelTask as RmfCancelTask
from rmf_task_msgs.srv import SubmitTask as RmfSubmitTask

from api_server.repositories.rmf import RmfRepository

from ... import models as mdl
from ...gateway import RmfGateway
from ...models.tasks import TaskProgress, TaskSummary


class DispatcherClient:
    def __init__(self, rmf_repo: RmfRepository, rmf_gateway: RmfGateway):
        self.rmf_repo = rmf_repo
        self.rmf_gateway = rmf_gateway

    async def submit_task_request(self, req_msg: RmfSubmitTask.Request):
        """
        Task Submission - This function will trigger a ros srv call to the
        dispatcher node, and return a response. Function will return a Task ID.
        Raises "HTTPException" if service call fails.
        """
        resp: RmfSubmitTask.Response = await self.rmf_gateway.call_service(
            self.rmf_gateway.submit_task_srv, req_msg
        )
        return resp

    def get_sim_time(self):
        if self.rmf_gateway.get_parameter("use_sim_time").value:
            sim_time = self.rmf_gateway.get_clock().now().to_msg()
        else:
            sim_time = None
        return sim_time

    async def cancel_task_request(self, task: mdl.CancelTask) -> bool:
        """
        Cancel Task - This function will trigger a ros srv call to the
        dispatcher node, and return a response.
        Raises "HTTPException" if service call fails.
        """
        req = RmfCancelTask.Request()
        req.task_id = task.task_id
        response = await self.rmf_gateway.call_service(
            self.rmf_gateway.cancel_task_srv, req
        )
        return response.success

    def convert_task_status_msg(self, task_summary: TaskSummary) -> TaskProgress:
        """
        convert rmf task summary msg to a task
        """
        now = self.rmf_gateway.get_clock().now().to_msg().sec  # only use sec
        # Generate a progress percentage
        duration = abs(task_summary.end_time.sec - task_summary.start_time.sec)
        # check if is completed
        if task_summary.state == RmfTaskSummary.STATE_COMPLETED:
            progress = "100%"
        # check if it state is queued/cancelled
        elif duration == 0 or (task_summary.state in [0, 4]):
            progress = "0%"
        else:
            percent = int(100 * (now - task_summary.start_time.sec) / float(duration))
            if percent < 0:
                progress = "0%"
            elif percent > 100:
                progress = "Delayed"
            else:
                progress = f"{percent}%"
        return TaskProgress(task_summary=task_summary, progress=progress)
