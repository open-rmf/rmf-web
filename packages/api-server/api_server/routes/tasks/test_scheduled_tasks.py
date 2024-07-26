import asyncio
import unittest
import unittest.mock
from datetime import date, datetime, timedelta, timezone
from zoneinfo import ZoneInfo

import pydantic
import schedule
from freezegun import freeze_time

from api_server.app import app
from api_server.app_config import app_config
from api_server.models import TaskRequest
from api_server.models.tasks import ScheduledTask, ScheduledTaskSchedule
from api_server.routes.tasks.scheduled_tasks import PostScheduledTaskRequest
from api_server.scheduler import get_scheduler
from api_server.test import AppFixture


class TestScheduledTasksRoute(AppFixture):
    def test_scheduled_task_crud(self):
        task_until = datetime.now() + timedelta(days=30)

        scheduled_task = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="00:00",
                ),
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Monday,
                    at="00:00",
                ),
            ],
            start_from=datetime.fromtimestamp(1000),
            until=task_until,
        )
        resp = self.client.post(
            "/scheduled_tasks", content=scheduled_task.model_dump_json()
        )
        self.assertEqual(201, resp.status_code, resp.json())
        task1 = ScheduledTask.model_validate_json(resp.content)
        self.assertEqual(len(task1.schedules), 2, task1)

        scheduled_task_2 = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="00:00",
                ),
            ],
            start_from=datetime.fromtimestamp(2000),
            until=task_until,
        )
        resp = self.client.post(
            "/scheduled_tasks", content=scheduled_task_2.model_dump_json()
        )
        self.assertEqual(201, resp.status_code, resp.json())
        task2 = ScheduledTask.model_validate_json(resp.content)
        self.assertEqual(len(task2.schedules), 1, task2)

        # check each task id only appears once
        until_after = task_until.timestamp()
        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={until_after}"
        )
        self.assertEqual(200, resp.status_code)
        ScheduledTaskList = pydantic.TypeAdapter(list[ScheduledTask])
        results = ScheduledTaskList.validate_json(resp.content)
        unique_ids = set(x.id for x in results)
        self.assertEqual(
            len(results), len(unique_ids), "one or more task appears multiple times"
        )
        self.assertIn(task1.id, unique_ids)
        self.assertIn(task2.id, unique_ids)

        resp = self.client.get(
            f"/scheduled_tasks?start_before=1000&until_after={until_after}"
        )
        self.assertEqual(200, resp.status_code, resp.json())
        results = ScheduledTaskList.validate_json(resp.content)
        task_ids = [x.id for x in results]
        self.assertIn(task1.id, task_ids)
        # task2 starts after `start_before`` so should not be included
        self.assertNotIn(task2.id, task_ids)

        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={until_after+1}"
        )
        self.assertEqual(200, resp.status_code, resp.json())
        results = ScheduledTaskList.validate_json(resp.content)
        task_ids = [x.id for x in results]
        # neither task should be returned as they stop before `until_after`
        self.assertNotIn(task1.id, task_ids)
        self.assertNotIn(task2.id, task_ids)

        resp = self.client.get(f"/scheduled_tasks/{task1.id}")
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks/{task2.id}")
        self.assertEqual(200, resp.status_code)

        resp = self.client.delete(f"/scheduled_tasks/{task1.id}")
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks/{task1.id}")
        self.assertEqual(404, resp.status_code)
        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until}"
        )
        results = ScheduledTaskList.validate_json(resp.content)
        task_ids = [x.id for x in results]
        self.assertNotIn(task1.id, task_ids)
        # task 2 should not be deleted
        self.assertIn(task2.id, task_ids)

        resp = self.client.delete(f"/scheduled_tasks/{task2.id}")
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks/{task2.id}")
        self.assertEqual(404, resp.status_code)
        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until}"
        )
        results = ScheduledTaskList.validate_json(resp.content)
        task_ids = [x.id for x in results]
        self.assertNotIn(task1.id, task_ids)
        self.assertNotIn(task2.id, task_ids)

    def test_cannot_create_task_that_never_runs(self):
        scheduled_task = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "start_from": 0,
                    "until": 0,
                    "period": "day",
                }
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task)
        self.assertEqual(422, resp.status_code)

    def test_get_scheduled_tasks_return_indefinite_tasks(self):
        scheduled_task = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [{"period": "day", "at": "00:00"}],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task)
        self.assertEqual(201, resp.status_code, resp.json())
        task = resp.json()

        resp = self.client.get("/scheduled_tasks?start_before=0&until_after=0")
        self.assertEqual(200, resp.status_code, resp.json())
        tasks = {x["id"]: x for x in resp.json()}
        self.assertIn(task["id"], tasks)

    def test_delete_scheduled_task(self):
        schedule_date_str = "2124-02-10T10:00:00+08:00"
        scheduled_task_description = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "period": "day",
                    "start_from": schedule_date_str,
                    "at": "18:00",
                },
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)
        schedule_task_id = scheduled_task["id"]

        # delete scheduled task
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")

        # the scheduled task is no more
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(404, resp.status_code, resp.json())

    def test_delete_scheduled_task_event_at_eight_am(self):
        scheduled_date_str = "2124-02-10T08:00:00+08:00"
        scheduled_task_description = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "period": "day",
                    "start_from": scheduled_date_str,
                    "at": "08:00",
                },
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # delete a single event from this schedule, for 0800 GMT+8 11th February 2124,
        # which is 0000 UTC 11th February 2124.
        schedule_task_id = scheduled_task["id"]
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/except_date",
            json={"except_date": "2124-02-11T00:00:00+00:00"},
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # the except_date should be 11th February 2124
        self.assertEqual(len(scheduled_task["except_dates"]), 1, scheduled_task)
        self.assertEqual(
            scheduled_task["except_dates"][0], "2124-02-11", scheduled_task
        )

        # delete a single event from this schedule, for 0800 GMT+8 12th February 2124
        schedule_task_id = scheduled_task["id"]
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/except_date",
            json={"except_date": "2124-02-12T08:00:00+08:00"},
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # the except_date should now have both 11th and 12th February 2124
        self.assertEqual(len(scheduled_task["except_dates"]), 2, scheduled_task)
        self.assertTrue("2124-02-11" in scheduled_task["except_dates"], scheduled_task)
        self.assertTrue("2124-02-12" in scheduled_task["except_dates"], scheduled_task)

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")

    def test_delete_scheduled_task_event_after_eight_am(self):
        # testing server operates in GMT+8, while all the dates are
        # transmitted to the server using UTC.

        # 0801 GMT+8 10th February 2124, is 0001 UTC 10th February 2124
        scheduled_date_str = "2124-02-10T00:01:00+00:00"
        scheduled_task_description = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "period": "day",
                    "start_from": scheduled_date_str,
                    "at": "08:01",
                },
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # delete a single event from this schedule, for 0801 GMT+8 11th February 2124,
        # which is 0001 UTC 11th February 2124.
        schedule_task_id = scheduled_task["id"]
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/except_date",
            json={"except_date": "2124-02-11T00:01:00+00:00"},
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # the except_date should be 11th February 2124
        self.assertEqual(len(scheduled_task["except_dates"]), 1, scheduled_task)
        self.assertEqual(
            scheduled_task["except_dates"][0], "2124-02-11", scheduled_task
        )

        # delete a single event from this schedule, for 0801 GMT+8 12th February 2124
        schedule_task_id = scheduled_task["id"]
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/except_date",
            json={"except_date": "2124-02-12T08:01:00+08:00"},
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # the except_date should now have both 11th and 12th February 2124
        self.assertEqual(len(scheduled_task["except_dates"]), 2, scheduled_task)
        self.assertTrue("2124-02-11" in scheduled_task["except_dates"], scheduled_task)
        self.assertTrue("2124-02-12" in scheduled_task["except_dates"], scheduled_task)

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")

    def test_delete_scheduled_task_event_before_eight_am(self):
        scheduled_date_str = "2124-02-09T07:59:00+08:00"
        scheduled_task_description = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "period": "day",
                    "start_from": scheduled_date_str,
                    "at": "07:59",
                }
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # delete a single event from this schedule, for 0759 GMT+8 11th February 2124,
        # which is 2359 UTC 10th February 2124.
        schedule_task_id = scheduled_task["id"]
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/except_date",
            json={"except_date": "2124-02-10T23:59:00+00:00"},
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # since the server is operating in GMT+8, the except_date should be
        # 11th February 2124, and not 10th February 2124
        self.assertEqual(len(scheduled_task["except_dates"]), 1, scheduled_task)
        self.assertEqual(
            scheduled_task["except_dates"][0], "2124-02-11", scheduled_task
        )

        # delete a single event from this schedule, for 0759 GMT+8 12th February 2124
        schedule_task_id = scheduled_task["id"]
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/except_date",
            json={"except_date": "2124-02-12T07:59:00+08:00"},
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # since the server is operating in GMT+8, the except_date should have
        # both 11th and 12th February 2124
        self.assertEqual(len(scheduled_task["except_dates"]), 2, scheduled_task)
        self.assertTrue("2124-02-11" in scheduled_task["except_dates"], scheduled_task)
        self.assertTrue("2124-02-12" in scheduled_task["except_dates"], scheduled_task)

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")

    def test_delete_scheduled_task_event_at_local_midnight(self):
        scheduled_date_str = "2124-02-12T00:00:00+08:00"
        scheduled_task_description = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "period": "day",
                    "start_from": scheduled_date_str,
                    "at": "00:00",
                }
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # delete a single event from this schedule, for 0000 GMT+8 13th February 2124,
        # which is 1600 UTC 12th February 2124.
        schedule_task_id = scheduled_task["id"]
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/except_date",
            json={"except_date": "2124-02-12T16:00:00+00:00"},
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # since the server is operating in GMT+8, the except_date should be
        # 13th February 2124, and not 12th February 2124
        self.assertEqual(len(scheduled_task["except_dates"]), 1, scheduled_task)
        self.assertEqual(
            scheduled_task["except_dates"][0], "2124-02-13", scheduled_task
        )

        # delete a single event from this schedule, for 0000 GMT+8 14th February 2124
        schedule_task_id = scheduled_task["id"]
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/except_date",
            json={"except_date": "2124-02-14T00:00:00+08:00"},
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # The except_date should be both 13th and 14th February 2124
        self.assertEqual(len(scheduled_task["except_dates"]), 2, scheduled_task)
        self.assertTrue("2124-02-13" in scheduled_task["except_dates"], scheduled_task)
        self.assertTrue("2124-02-14" in scheduled_task["except_dates"], scheduled_task)

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")

    def test_edit_scheduled_task(self):
        task_request = TaskRequest(
            category="test_category",
            description="test_description",
        )
        task_schedules = [
            ScheduledTaskSchedule(
                period=ScheduledTaskSchedule.Period.Day,
                at="16:00",
            )
        ]

        scheduled_task_description = PostScheduledTaskRequest(
            task_request=task_request,
            start_from=datetime.fromisoformat("2124-02-10T16:00:00+08:00"),
            schedules=task_schedules,
        )
        resp = self.client.post(
            "/scheduled_tasks", content=scheduled_task_description.model_dump_json()
        )
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = ScheduledTask.model_validate_json(resp.content)
        self.assertEqual(len(scheduled_task.schedules), 1, scheduled_task)
        schedule_task_id = scheduled_task.id
        schedules = scheduled_task.schedules

        # update task description
        updated_task_request = TaskRequest(
            category="test_category_updated",
            description="test_description_updated",
        )
        scheduled_task_description.task_request = updated_task_request
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update",
            content=scheduled_task_description.model_dump_json(),
        )
        self.assertEqual(200, resp.status_code, resp.json())

        # compare updated scheduled task description
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = ScheduledTask.model_validate_json(resp.content)
        self.assertEqual(updated_scheduled_task.id, schedule_task_id)
        self.assertEqual(
            updated_scheduled_task.task_request,
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertEqual(
            updated_scheduled_task.schedules, schedules, updated_scheduled_task
        )

        # update task schedules in UTC
        updated_scheduled_date = datetime.fromisoformat("2124-02-10T12:00:00+00:00")
        updated_task_schedules = [
            ScheduledTaskSchedule(
                period=ScheduledTaskSchedule.Period.Day,
                at="20:00",
            )
        ]
        scheduled_task_description.start_from = updated_scheduled_date
        scheduled_task_description.schedules = updated_task_schedules
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update",
            content=scheduled_task_description.model_dump_json(),
        )
        self.assertEqual(200, resp.status_code, resp.json())

        # compare updated scheduled task schedules
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = ScheduledTask.model_validate_json(resp.content)
        self.assertEqual(
            updated_scheduled_task.task_request,
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertNotEqual(
            updated_scheduled_task.schedules, schedules, updated_scheduled_task
        )
        self.assertEqual(
            len(updated_scheduled_task.schedules), 1, updated_scheduled_task
        )
        updated_schedule = updated_scheduled_task.schedules[0]
        self.assertEqual(
            updated_scheduled_task.start_from,
            datetime.fromisoformat("2124-02-10T12:00:00+00:00"),
            updated_scheduled_task.start_from,
        )
        self.assertEqual(updated_schedule.at, "20:00", updated_schedule)

        # update task schedules in GMT+8
        updated_scheduled_date = datetime.fromisoformat("2124-02-10T21:00:00+08:00")
        scheduled_task_description.start_from = updated_scheduled_date
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update",
            content=scheduled_task_description.model_dump_json(),
        )
        self.assertEqual(200, resp.status_code, resp.json())

        # compare updated scheduled task schedules
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = ScheduledTask.model_validate_json(resp.content)
        self.assertEqual(
            updated_scheduled_task.task_request,
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertEqual(
            len(updated_scheduled_task.schedules), 1, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task.start_from,
            updated_scheduled_date,
            updated_schedule,
        )
        updated_schedule = updated_scheduled_task.schedules[0]

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")

    def test_edit_scheduled_task_event(self):
        task_request = TaskRequest(
            category="test_category",
            description="test_description",
        )
        task_schedules = [
            ScheduledTaskSchedule(
                period=ScheduledTaskSchedule.Period.Day,
                at="10:00",
            )
        ]

        scheduled_task_description = PostScheduledTaskRequest(
            task_request=task_request,
            start_from=datetime.fromisoformat("2124-02-10T10:00:00+08:00"),
            schedules=task_schedules,
        )
        resp = self.client.post(
            "/scheduled_tasks", content=scheduled_task_description.model_dump_json()
        )
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = ScheduledTask.model_validate_json(resp.content)
        self.assertEqual(len(scheduled_task.schedules), 1, scheduled_task)
        schedule_task_id = scheduled_task.id

        # update task description, with except date on 12th UTC, which will be 13th GMT+8
        updated_task_request = TaskRequest(
            category="test_category_updated",
            description="test_description_updated",
        )
        scheduled_task_description.task_request = updated_task_request
        scheduled_task_description.except_dates = [
            datetime.fromisoformat("2124-02-12T16:00:00+00:00")
        ]

        # this will be accompanied by an until date of 23:59 12th UTC
        updated_task_schedule = task_schedules[0]
        scheduled_task_description.schedules[0] = updated_task_schedule
        until = datetime.fromisoformat("2124-02-12T23:59:59+00:00")
        scheduled_task_description.until = until

        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update",
            content=scheduled_task_description.model_dump_json(),
        )
        self.assertEqual(200, resp.status_code, resp.json())

        # compare updated scheduled task schedules
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = ScheduledTask.model_validate_json(resp.content)
        self.assertEqual(
            updated_scheduled_task.task_request,
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertEqual(
            updated_scheduled_task.schedules,
            scheduled_task_description.schedules,
            updated_scheduled_task,
        )
        self.assertEqual(
            len(updated_scheduled_task.except_dates), 1, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task.except_dates[0],
            date.fromisoformat("2124-02-13"),
            updated_scheduled_task,
        )

        # update task description, with except date on 12th GMT+8
        updated_task_request = TaskRequest(
            category="test_category_updated",
            description="test_description_updated",
        )
        scheduled_task_description.task_request = updated_task_request
        scheduled_task_description.except_dates = [
            datetime.fromisoformat("2124-02-12T07:59:00+08:00")
        ]

        # this will be accompanied by an until date of 23:59 12th GMT+8
        updated_task_schedule = task_schedules[0]
        scheduled_task_description.schedules[0] = updated_task_schedule
        until = datetime.fromisoformat("2124-02-12T23:59:59+00:00")
        scheduled_task_description.until = until

        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update",
            content=scheduled_task_description.model_dump_json(),
        )
        self.assertEqual(200, resp.status_code, resp.json())

        # compare updated scheduled task schedules
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = ScheduledTask.model_validate_json(resp.content)
        self.assertEqual(
            updated_scheduled_task.task_request,
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertEqual(
            updated_scheduled_task.schedules,
            scheduled_task_description.schedules,
            updated_scheduled_task,
        )
        self.assertEqual(
            len(updated_scheduled_task.except_dates), 1, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task.except_dates[0],
            date.fromisoformat("2124-02-12"),
            updated_scheduled_task,
        )

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")


class TestScheduledTaskExecution(AppFixture):
    def setUp(self):
        super().setUp()
        self.scheduler = schedule.Scheduler()
        app.dependency_overrides[get_scheduler] = lambda: self.scheduler

        self.serverTz = ZoneInfo(app_config.timezone)
        self._patcher = unittest.mock.patch(
            "api_server.routes.tasks.scheduled_tasks.post_dispatch_task"
        )
        self.dispatch_task_mock = self._patcher.start()

    def tearDown(self):
        self._patcher.stop()
        app.dependency_overrides = {}

    def run_pending_jobs(self):
        async def run():
            before = asyncio.all_tasks()
            self.scheduler.run_pending()
            after = asyncio.all_tasks()
            job_tasks = after.difference(before)
            for x in job_tasks:
                await x

        self.portal.call(run)

    def test_schedule_runs_at_midnight(self):
        scheduled_task = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="00:00",
                ),
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Monday,
                    at="00:00",
                ),
            ],
        )

        # server time is Asia/Singapore, which would be 1970-01-01T07:30:00+07:30.
        # The first run would be in 1970-01-02T00:00:00+07:30
        with freeze_time(datetime.fromtimestamp(0)):
            resp = self.client.post(
                "/scheduled_tasks", content=scheduled_task.model_dump_json()
            )
            self.assertEqual(201, resp.status_code, resp.json())
        with freeze_time(datetime(1970, 1, 1, 23, 59, 59, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_not_called()
        with freeze_time(datetime(1970, 1, 2, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()
        dt = datetime.fromtimestamp(0, tz=self.serverTz)
        days_to_next_monday = (7 - dt.weekday()) % 7
        next_monday = dt + timedelta(days=days_to_next_monday)
        with freeze_time(next_monday):
            self.run_pending_jobs()
            # Both the daily and every monday schedule should be called.
            self.assertEqual(self.dispatch_task_mock.call_count, 3)

    def test_schedule_runs_start_from(self):
        scheduled_task = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="00:00",
                ),
            ],
            start_from=datetime(1970, 1, 3, tzinfo=self.serverTz),
        )

        with freeze_time(datetime.fromtimestamp(0)):
            resp = self.client.post(
                "/scheduled_tasks", content=scheduled_task.model_dump_json()
            )
            self.assertEqual(201, resp.status_code, resp.json())
        with freeze_time(datetime(1970, 1, 2, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            # 1970-01-02 is before the start_from
            self.dispatch_task_mock.assert_not_called()
        with freeze_time(datetime(1970, 1, 3, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()

    def test_schedule_runs_start_from_utc(self):
        scheduled_task = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="00:00",
                ),
            ],
            start_from=datetime(1970, 1, 3, tzinfo=timezone.utc),
        )

        with freeze_time(datetime.fromtimestamp(0)):
            resp = self.client.post(
                "/scheduled_tasks", content=scheduled_task.model_dump_json()
            )
            self.assertEqual(201, resp.status_code, resp.json())
        with freeze_time(datetime(1970, 1, 2, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            # 1970-01-02 is before the start_from
            self.dispatch_task_mock.assert_not_called()
        with freeze_time(datetime(1970, 1, 3, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            # still should not be called because start_from is 1970-01-03T07:30+07:30
            # when converted to server time
            self.dispatch_task_mock.assert_not_called()
        with freeze_time(datetime(1970, 1, 4, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()

    def test_schedule_runs_until(self):
        scheduled_task = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="00:00",
                ),
            ],
            until=datetime(1970, 1, 2, tzinfo=self.serverTz),
        )

        with freeze_time(datetime.fromtimestamp(0)):
            resp = self.client.post(
                "/scheduled_tasks", content=scheduled_task.model_dump_json()
            )
            self.assertEqual(201, resp.status_code, resp.json())
        with freeze_time(datetime(1970, 1, 2, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()
        with freeze_time(datetime(1970, 1, 3, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            # schedule should have stopped
            self.dispatch_task_mock.assert_called_once()

    def test_schedule_runs_until_utc(self):
        scheduled_task = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="00:00",
                ),
            ],
            until=datetime(1970, 1, 1, 16, 30, tzinfo=timezone.utc),
        )

        with freeze_time(datetime.fromtimestamp(0)):
            resp = self.client.post(
                "/scheduled_tasks", content=scheduled_task.model_dump_json()
            )
            self.assertEqual(201, resp.status_code, resp.json())
        with freeze_time(datetime(1970, 1, 2, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            # `until` when converted to server time is 1970-01-02T00:00+07:30
            # so it should still be called
            self.dispatch_task_mock.assert_called_once()
        with freeze_time(datetime(1970, 1, 3, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            # schedule should have stopped
            self.dispatch_task_mock.assert_called_once()

    def test_schedule_runs_except_date(self):
        scheduled_task = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="00:00",
                ),
            ],
            except_dates=[datetime(1970, 1, 3, tzinfo=self.serverTz)],
        )

        with freeze_time(datetime.fromtimestamp(0)):
            resp = self.client.post(
                "/scheduled_tasks", content=scheduled_task.model_dump_json()
            )
            self.assertEqual(201, resp.status_code, resp.json())
        with freeze_time(datetime(1970, 1, 2, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()
        with freeze_time(datetime(1970, 1, 3, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()
        with freeze_time(datetime(1970, 1, 4, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.assertEqual(self.dispatch_task_mock.call_count, 2)

    def test_schedule_runs_except_date_utc(self):
        scheduled_task = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="00:00",
                ),
            ],
            # this will be 1970-01-03 on server time
            except_dates=[datetime(1970, 1, 2, 16, 30, tzinfo=timezone.utc)],
        )

        with freeze_time(datetime.fromtimestamp(0)):
            resp = self.client.post(
                "/scheduled_tasks", content=scheduled_task.model_dump_json()
            )
            self.assertEqual(201, resp.status_code, resp.json())
        with freeze_time(datetime(1970, 1, 2, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()
        with freeze_time(datetime(1970, 1, 3, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()
        with freeze_time(datetime(1970, 1, 4, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.assertEqual(self.dispatch_task_mock.call_count, 2)

    def test_schedule_runs_except_date_multiple(self):
        scheduled_task = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="00:00",
                ),
            ],
            except_dates=[
                datetime(1970, 1, 4, tzinfo=self.serverTz),
                # this will be 1970-01-03 on server time
                datetime(1970, 1, 2, 16, 30, tzinfo=timezone.utc),
            ],
        )

        with freeze_time(datetime.fromtimestamp(0)):
            resp = self.client.post(
                "/scheduled_tasks", content=scheduled_task.model_dump_json()
            )
            self.assertEqual(201, resp.status_code, resp.json())
        with freeze_time(datetime(1970, 1, 2, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()
        with freeze_time(datetime(1970, 1, 3, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()
        with freeze_time(datetime(1970, 1, 4, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()
        with freeze_time(datetime(1970, 1, 5, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.assertEqual(self.dispatch_task_mock.call_count, 2)

    def test_schedule_delete_before_run(self):
        scheduled_task = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="00:00",
                ),
            ],
        )

        with freeze_time(datetime.fromtimestamp(0)):
            resp = self.client.post(
                "/scheduled_tasks", content=scheduled_task.model_dump_json()
            )
            self.assertEqual(201, resp.status_code, resp.json())
            scheduled_task = ScheduledTask.model_validate_json(resp.content)
        with freeze_time(datetime(1970, 1, 1, 23, 59, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_not_called()
        self.client.delete(f"/scheduled_tasks/{scheduled_task.id}")
        with freeze_time(datetime(1970, 1, 2, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_not_called()

    def test_schedule_update_before_run(self):
        scheduled_task_req = PostScheduledTaskRequest(
            task_request=TaskRequest(
                category="test",
                description="test",
            ),
            schedules=[
                ScheduledTaskSchedule(
                    period=ScheduledTaskSchedule.Period.Day,
                    at="01:00",
                ),
            ],
        )

        with freeze_time(datetime.fromtimestamp(0)):
            resp = self.client.post(
                "/scheduled_tasks", content=scheduled_task_req.model_dump_json()
            )
            self.assertEqual(201, resp.status_code, resp.json())
            scheduled_task = ScheduledTask.model_validate_json(resp.content)
        scheduled_task_req.schedules[0].at = "00:00"
        scheduled_task_req.start_from = datetime(1970, 1, 3, tzinfo=self.serverTz)
        scheduled_task_req.until = datetime(1970, 1, 5, tzinfo=self.serverTz)
        scheduled_task_req.except_dates = [datetime(1970, 1, 4, tzinfo=self.serverTz)]
        with freeze_time(datetime(1970, 1, 1, 23, 59, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_not_called()
            resp = self.client.post(
                f"/scheduled_tasks/{scheduled_task.id}/update",
                content=scheduled_task_req.model_dump_json(),
            )
            self.assertEqual(200, resp.status_code, resp.json())
        # before start from
        with freeze_time(datetime(1970, 1, 2, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_not_called()
        with freeze_time(datetime(1970, 1, 3, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()
        # except date
        with freeze_time(datetime(1970, 1, 4, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.dispatch_task_mock.assert_called_once()
        with freeze_time(datetime(1970, 1, 5, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.assertEqual(self.dispatch_task_mock.call_count, 2)
        # after until time
        with freeze_time(datetime(1970, 1, 6, tzinfo=self.serverTz)):
            self.run_pending_jobs()
            self.assertEqual(self.dispatch_task_mock.call_count, 2)
