import asyncio
from typing import Sequence
from unittest.mock import AsyncMock, Mock

from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary
from rmf_task_msgs.msg import TaskType as RmfTaskType
from rmf_task_msgs.srv import CancelTask as RmfCancelTask
from rmf_task_msgs.srv import SubmitTask as RmfSubmitTask

from api_server.models import CancelTask, CleanTaskDescription, SubmitTask, TaskSummary
from api_server.models import tortoise_models as ttm
from api_server.permissions import RmfAction
from api_server.test.test_fixtures import AppFixture


class TasksFixture(AppFixture):
    async def asyncSetUp(self):
        await super().asyncSetUp()
        mock = AsyncMock()
        task_counter = 0

        def mock_submit_task(_req):
            nonlocal task_counter
            task_counter += 1
            return RmfSubmitTask.Response(
                task_id=f"test_task_{task_counter}", success=True
            )

        mock.side_effect = mock_submit_task
        self.app.rmf_gateway.submit_task = mock

        def mock_cancel_task(_req):
            return RmfCancelTask.Response(success=True)

        mock = AsyncMock()
        mock.side_effect = mock_cancel_task
        self.app.rmf_gateway.cancel_task = mock

    async def save_tasks(self, tasks: Sequence[TaskSummary], authz_grp: str):
        await asyncio.gather(
            *(ttm.TaskSummary.save_pydantic(t, authz_grp) for t in tasks)
        )


class TestTasksRoute(TasksFixture):
    async def test_permissions(self):
        user1 = await self.create_user(True)
        user2 = await self.create_user()
        role1 = await self.create_role()
        await self.add_permission(role1, RmfAction.TaskRead)
        await self.add_permission(role1, RmfAction.TaskSubmit)
        await self.add_permission(role1, RmfAction.TaskCancel)
        await self.assign_role(user2, role1)
        user3 = await self.create_user()
        user4 = await self.create_user()
        role2 = await self.create_role()
        await self.add_permission(role2, RmfAction.TaskRead, "group1")
        await self.assign_role(user4, role2)
        self.set_user(user1)

        # admin can submit tasks
        submit_task = SubmitTask(
            task_type=RmfTaskType.TYPE_CLEAN,
            start_time=0,
            description=CleanTaskDescription(cleaning_zone="zone_1"),
            priority=0,
        )
        resp = await self.client.post("/tasks/submit_task", data=submit_task.json())
        self.assertEqual(200, resp.status_code)
        submitted_task_1 = resp.json()["task_id"]

        # user with task submit role can submit task
        self.set_user(user2)
        resp = await self.client.post("/tasks/submit_task", data=submit_task.json())
        self.assertEqual(200, resp.status_code)
        submitted_task_2 = resp.json()["task_id"]

        # user without permission cannot submit task
        self.set_user(user3)
        resp = await self.client.post("/tasks/submit_task", data=submit_task.json())
        self.assertEqual(403, resp.status_code)

        # admin can see submitted task
        self.set_user(user1)
        resp = await self.client.get("/tasks")
        tasks = resp.json()
        self.assertEqual(2, len(tasks))
        resp = await self.client.get(f"/tasks/{submitted_task_1}/summary")
        self.assertEqual(200, resp.status_code)

        # user with permission can see submitted task
        self.set_user(user2)
        resp = await self.client.get("/tasks")
        tasks = resp.json()
        self.assertEqual(2, len(tasks))
        resp = await self.client.get(f"/tasks/{submitted_task_1}/summary")
        self.assertEqual(200, resp.status_code)

        # user without permission cannot see submitted task
        self.set_user(user3)
        resp = await self.client.get("/tasks")
        tasks = resp.json()
        self.assertEqual(0, len(tasks))
        resp = await self.client.get(f"/tasks/{submitted_task_1}/summary")
        self.assertEqual(404, resp.status_code)

        #
        # Cancelling tasks
        #

        # user without permission cannot cancel task
        self.set_user(user4)
        resp = await self.client.post(
            "/tasks/cancel_task",
            data=CancelTask(task_id=submitted_task_2).json(),
        )
        self.assertEqual(403, resp.status_code)

        # user with permission can cancel task
        self.set_user(user2)
        resp = await self.client.post(
            "/tasks/cancel_task",
            data=CancelTask(task_id=submitted_task_2).json(),
        )
        self.assertEqual(200, resp.status_code)

        # admin can cancel task
        self.set_user(user1)
        resp = await self.client.post(
            "/tasks/cancel_task",
            data=CancelTask(task_id=submitted_task_1).json(),
        )
        self.assertEqual(200, resp.status_code)

    async def test_query_tasks(self):
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

        await self.save_tasks(dataset, "test_group")

        resp = await self.client.get("/tasks?task_id=task_1,task_2")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json
        self.assertEqual(len(items), 2)

        resp = await self.client.get("/tasks?fleet_name=fleet_1")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["summary"]["task_id"], "task_1")
        self.assertEqual(items[0]["summary"]["fleet_name"], "fleet_1")

        resp = await self.client.get("/tasks?robot_name=robot_1")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["summary"]["task_id"], "task_1")
        self.assertEqual(items[0]["summary"]["robot_name"], "robot_1")

        resp = await self.client.get("/tasks?state=completed")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["summary"]["task_id"], "task_1")
        self.assertEqual(items[0]["summary"]["state"], RmfTaskSummary.STATE_COMPLETED)

        resp = await self.client.get("/tasks?task_type=loop")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["summary"]["task_id"], "task_1")
        self.assertEqual(
            items[0]["summary"]["task_profile"]["description"]["task_type"]["type"],
            RmfTaskType.TYPE_LOOP,
        )

        resp = await self.client.get("/tasks?priority=0")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["summary"]["task_id"], "task_1")

        resp = await self.client.get("/tasks?submission_time_since=4000")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["summary"]["task_id"], "task_2")

        resp = await self.client.get("/tasks?start_time_since=5000")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["summary"]["task_id"], "task_2")

        resp = await self.client.get("/tasks?end_time_since=6000")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["summary"]["task_id"], "task_2")

        # test no match
        resp = await self.client.get("/tasks?fleet_name=fleet_1&start_time_since=5000")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json
        self.assertEqual(len(items), 0)

        # no query returns everything
        resp = await self.client.get("/tasks")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        items = resp_json
        self.assertEqual(len(items), 2)
