from fastapi.exceptions import HTTPException
from rmf_task_msgs.srv import CancelTask as RmfCancelTask
from rmf_task_msgs.srv import SubmitTask as RmfSubmitTask

from api_server import models as mdl
from api_server.gateway import RmfGateway
from api_server.models import tortoise_models as ttm
from api_server.permissions import Enforcer, RmfAction


class DispatcherClient:
    def __init__(self, rmf_gateway: RmfGateway):
        self.rmf_gateway = rmf_gateway

    # pylint: disable=unused-argument
    @staticmethod
    async def _get_submit_task_authz_grp(task: RmfSubmitTask.Request):
        # TODO
        return ""

    async def submit_task_request(self, user: mdl.User, req_msg: RmfSubmitTask.Request):
        """
        Task Submission - This function will trigger a ros srv call to the
        dispatcher node, and return a response. Function will return a Task ID.
        Raises "HTTPException" if service call fails.
        """
        authz_grp = await DispatcherClient._get_submit_task_authz_grp(req_msg)
        if not await Enforcer.is_authorized(user, authz_grp, RmfAction.TaskSubmit):
            raise HTTPException(403)

        resp = await self.rmf_gateway.submit_task(req_msg)

        await ttm.TaskSummary.update_or_create(
            {"authz_grp": authz_grp, "data": {"task_id": resp.task_id}},
            id_=resp.task_id,
        )

        return resp

    # pylint: disable=unused-argument
    @staticmethod
    async def _get_cancel_task_authz_grp(task: mdl.CancelTask):
        # TODO
        return ""

    async def cancel_task_request(self, task: mdl.CancelTask, user: mdl.User) -> bool:
        """
        Cancel Task - This function will trigger a ros srv call to the
        dispatcher node, and return a response.
        Raises "HTTPException" if service call fails.
        """
        authz_grp = await DispatcherClient._get_cancel_task_authz_grp(task)
        authorized = await Enforcer.is_authorized(user, authz_grp, RmfAction.TaskRead)
        if not authorized:
            raise HTTPException(403)

        req = RmfCancelTask.Request()
        req.requester = "rmf-server"
        req.task_id = task.task_id
        response = await self.rmf_gateway.cancel_task(req)
        if not response.success:
            raise HTTPException(500, response.message)
        return response.success
