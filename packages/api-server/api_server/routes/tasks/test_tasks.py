import asyncio
from typing import Optional, Sequence

from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary
from rmf_task_msgs.msg import TaskType as RmfTaskType
from rmf_task_msgs.srv import CancelTask as RmfCancelTask
from rmf_task_msgs.srv import SubmitTask as RmfSubmitTask

from ...models import CancelTask, CleanTaskDescription, SubmitTask, TaskSummary, User
from ...models import tortoise_models as ttm
from ...permissions import Enforcer, Permission, RmfRole
from ..test_fixtures import RouteFixture


def save_tasks(fixture: RouteFixture, tasks: Sequence[TaskSummary], user: User):
    async def save_data():
        ttm_tasks = await asyncio.gather(
            *(ttm.TaskSummary.save_pydantic(t, user) for t in tasks)
        )
        await asyncio.gather(
            *(
                Enforcer.save_permissions(t, user.groups, [Permission.Read])
                for t in ttm_tasks
            )
        )

    fixture.run_in_app_loop(save_data())


class TestTasksRoute(RouteFixture):
    def test_query_tasks(self):
        dataset = (
            TaskSummary(
                task_id="task_1",
                fleet_name="fleet_1",
                submission_time={"sec": 1000, "nanosec": 0},
                start_time={"sec": 2000, "nanosec": 0},
                end_time={"sec": 3000, "nanosec": 0},
                robot_name="robot_1",
                state=RmfTaskSummary.STATE_COMPLETED,
                task_profile={
                    "description": {
                        "task_type": {"type": RmfTaskType.TYPE_LOOP},
                        "priority": {"value": 0},
                    }
                },
            ),
            TaskSummary(
                task_id="task_2",
                fleet_name="fleet_2",
                submission_time={"sec": 4000, "nanosec": 0},
                start_time={"sec": 5000, "nanosec": 0},
                end_time={"sec": 6000, "nanosec": 0},
                robot_name="robot_2",
                state=RmfTaskSummary.STATE_ACTIVE,
                task_profile={
                    "description": {
                        "task_type": {"type": RmfTaskType.TYPE_DELIVERY},
                        "priority": {"value": 1},
                    }
                },
            ),
        )

        save_tasks(self, dataset, self.user)

        resp = self.session.get(f"{self.base_url}/tasks?task_id=task_1,task_2")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json["items"]
        self.assertEqual(len(items), 2)

        resp = self.session.get(f"{self.base_url}/tasks?fleet_name=fleet_1")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json["items"]
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["task_summary"]["task_id"], "task_1")
        self.assertEqual(items[0]["task_summary"]["fleet_name"], "fleet_1")

        resp = self.session.get(f"{self.base_url}/tasks?robot_name=robot_1")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json["items"]
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["task_summary"]["task_id"], "task_1")
        self.assertEqual(items[0]["task_summary"]["robot_name"], "robot_1")

        resp = self.session.get(f"{self.base_url}/tasks?state=completed")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json["items"]
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["task_summary"]["task_id"], "task_1")
        self.assertEqual(
            items[0]["task_summary"]["state"], RmfTaskSummary.STATE_COMPLETED
        )

        resp = self.session.get(f"{self.base_url}/tasks?task_type=loop")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json["items"]
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["task_summary"]["task_id"], "task_1")
        self.assertEqual(
            items[0]["task_summary"]["task_profile"]["description"]["task_type"][
                "type"
            ],
            RmfTaskType.TYPE_LOOP,
        )

        resp = self.session.get(f"{self.base_url}/tasks?priority=0")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json["items"]
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["task_summary"]["task_id"], "task_1")

        resp = self.session.get(f"{self.base_url}/tasks?submission_time_since=4000")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json["items"]
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["task_summary"]["task_id"], "task_2")

        resp = self.session.get(f"{self.base_url}/tasks?start_time_since=5000")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json["items"]
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["task_summary"]["task_id"], "task_2")

        resp = self.session.get(f"{self.base_url}/tasks?end_time_since=6000")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json["items"]
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["task_summary"]["task_id"], "task_2")

        # test no match
        resp = self.session.get(
            f"{self.base_url}/tasks?fleet_name=fleet_1&start_time_since=5000"
        )
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json["items"]
        self.assertEqual(len(items), 0)

        # no query returns everything
        resp = self.session.get(f"{self.base_url}/tasks")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json["items"]
        self.assertEqual(len(items), 2)

    def test_user_outside_groups_cannot_see_tasks(self):
        user = User(
            username="test_user",
            roles={RmfRole.TaskSubmit.value},
            groups={"rmf_test_group"},
        )

        save_tasks(self, [TaskSummary(task_id="task1")], user)

        user2 = User(
            username="test_user2",
        )
        self.set_user(user2)
        resp = self.session.get(f"{self.base_url}/tasks")
        self.assertEqual(200, resp.status_code)
        resp_json = resp.json()
        self.assertEqual(0, resp_json["total_count"])
        self.assertEqual(0, len(resp_json["items"]))


class TestSubmitTaskRoute(RouteFixture):
    def host_submit_service(self, resp: Optional[RmfSubmitTask.Response] = None):
        resp = resp or RmfSubmitTask.Response(success=True, task_id="test_task")
        return self.host_service_one(
            RmfSubmitTask,
            "submit_task",
            resp,
        )

    def test_smoke(self):
        # create a submit task request message
        task = SubmitTask(
            task_type=RmfTaskType.TYPE_CLEAN,
            start_time=0,
            description=CleanTaskDescription(cleaning_zone="zone_2"),
            priority=0,
        )
        fut = self.host_submit_service()
        resp = self.session.post(f"{self.base_url}/tasks/submit_task", data=task.json())
        self.assertEqual(resp.status_code, 200)
        ros_received: RmfSubmitTask.Request = fut.result(3)
        self.assertEqual(ros_received.requester, "rmf_server")

    def test_users_without_permissions_cannot_submit_task(self):
        task = SubmitTask(
            task_type=RmfTaskType.TYPE_CLEAN,
            start_time=0,
            description=CleanTaskDescription(cleaning_zone="zone_2"),
            priority=0,
        )
        self.set_user(User(username="user_with_no_roles"))
        resp = self.session.post(f"{self.base_url}/tasks/submit_task", data=task.json())
        self.assertEqual(resp.status_code, 401)


class TestCancelTaskRoute(RouteFixture):
    def test_smoke(self):
        save_tasks(self, [TaskSummary(task_id="test_task")], self.user)

        cancel_task = CancelTask(task_id="test_task")
        fut = self.host_service_one(
            RmfCancelTask, "cancel_task", RmfCancelTask.Response(success=True)
        )
        resp = self.session.post(
            f"{self.base_url}/tasks/cancel_task", data=cancel_task.json()
        )
        self.assertEqual(resp.status_code, 200)
        received: RmfCancelTask.Request = fut.result(3)
        self.assertEqual(received.task_id, "test_task")

    def test_cancel_task_failure(self):
        save_tasks(self, [TaskSummary(task_id="test_task")], self.user)

        cancel_task = CancelTask(task_id="test_task")
        fut = self.host_service_one(
            RmfCancelTask,
            "cancel_task",
            RmfCancelTask.Response(success=False, message="test error"),
        )
        resp = self.session.post(
            f"{self.base_url}/tasks/cancel_task", data=cancel_task.json()
        )
        self.assertEqual(500, resp.status_code)
        fut.result(3)
        self.assertEqual(resp.json()["detail"], "test error")

    def test_user_without_role_cannot_cancel_task(self):
        user = User(username="user_with_no_roles")
        self.set_user(user)

        save_tasks(self, [TaskSummary(task_id="test_task")], user)

        cancel_task = CancelTask(task_id="test_task")
        resp = self.session.post(
            f"{self.base_url}/tasks/cancel_task", data=cancel_task.json()
        )
        self.assertEqual(401, resp.status_code)

    def test_user_with_role_but_no_groups_cannot_cancel_task(self):
        user = User(
            username="test_user",
            roles={RmfRole.TaskSubmit.value},
            groups={"rmf_test_group"},
        )

        save_tasks(self, [TaskSummary(task_id="task1")], user)

        user2 = User(
            username="test_user2",
            roles={RmfRole.TaskCancel.value},
        )
        self.set_user(user2)
        cancel_task = CancelTask(task_id="task1")
        resp = self.session.post(
            f"{self.base_url}/tasks/cancel_task", data=cancel_task.json()
        )
        self.assertEqual(404, resp.status_code)
