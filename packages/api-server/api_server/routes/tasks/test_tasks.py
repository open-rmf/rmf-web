import asyncio

from rmf_task_msgs.msg import TaskType as RmfTaskType
from rmf_task_msgs.srv import CancelTask as RmfCancelTask
from rmf_task_msgs.srv import SubmitTask as RmfSubmitTask

from api_server.models.tasks import TaskSummary

from ...models import CancelTask, CleanTaskDescription, SubmitTask
from ..test_fixtures import RouteFixture


class TestTasksRoute(RouteFixture):
    def test_submit_task_request(self):
        # create a submit task request message
        task = SubmitTask(
            task_type=RmfTaskType.TYPE_CLEAN,
            start_time=0,
            description=CleanTaskDescription(cleaning_zone="zone_2"),
            priority=0,
        )
        fut = self.host_service_one(
            RmfSubmitTask, "submit_task", RmfSubmitTask.Response(success=True)
        )
        resp = self.session.post(f"{self.base_url}/tasks/submit_task", data=task.json())
        self.assertEqual(resp.status_code, 200)
        ros_received: RmfSubmitTask.Request = fut.result(3)
        self.assertEqual(ros_received.requester, "rmf_server")

    def test_cancel_task_request(self):
        cancel_task = CancelTask(task_id="test_task")
        fut = self.host_service_one(
            RmfCancelTask, "cancel_task", RmfCancelTask.Response()
        )
        resp = self.session.post(
            f"{self.base_url}/tasks/cancel_task", data=cancel_task.json()
        )
        self.assertEqual(resp.status_code, 200)
        received: RmfCancelTask.Request = fut.result(3)
        self.assertEqual(received.task_id, "test_task")

    def test_get_tasks(self):
        asyncio.get_event_loop().run_until_complete(
            self.app.rmf_repo.save_task_summary(TaskSummary(task_id="test_task"))
        )
        resp = self.session.get(f"{self.base_url}/tasks")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 1)
        self.assertEqual(resp_json[0]["task_summary"]["task_id"], "test_task")
