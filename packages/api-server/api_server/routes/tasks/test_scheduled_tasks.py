from datetime import datetime, timedelta
from urllib.parse import urlencode

from api_server.test import AppFixture


class TestScheduledTasksRoute(AppFixture):
    def test_scheduled_task_crud(self):
        task_until = (datetime.now() + timedelta(days=30)).timestamp()

        scheduled_task = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "period": "day",
                    "start_from": 1000,
                    "until": task_until,
                },
                {
                    "period": "day",
                    "start_from": 0,
                    "until": 999,
                },
                {
                    "period": "monday",
                    "start_from": 1000,
                    "until": task_until,
                },
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task)
        self.assertEqual(201, resp.status_code, resp.json())
        task1 = resp.json()
        self.assertEqual(len(task1["schedules"]), 3, task1)

        scheduled_task_2 = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "period": "day",
                    "start_from": 2000,
                    "until": task_until,
                },
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_2)
        self.assertEqual(201, resp.status_code, resp.json())
        task2 = resp.json()
        self.assertEqual(len(task2["schedules"]), 1, task2)

        # check each task id only appears once
        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until}"
        )
        self.assertEqual(200, resp.status_code)
        task_ids = [x["id"] for x in resp.json()]
        unique_ids = set(task_ids)
        self.assertEqual(
            len(task_ids), len(unique_ids), "one or more task appears multiple times"
        )

        resp = self.client.get(
            f"/scheduled_tasks?start_before=1000&until_after={task_until}"
        )
        self.assertEqual(200, resp.status_code, resp.json())
        tasks = {x["id"]: x for x in resp.json()}
        self.assertIn(task1["id"], tasks)
        # task2 starts after `start_before`` so should not be included
        self.assertNotIn(task2["id"], tasks)

        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until}"
        )
        self.assertEqual(200, resp.status_code, resp.json())
        after = resp.json()
        tasks = {x["id"]: x for x in after}
        self.assertIn(task1["id"], tasks)
        self.assertIn(task2["id"], tasks)

        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until+1}"
        )
        self.assertEqual(200, resp.status_code, resp.json())
        after = resp.json()
        tasks = {x["id"]: x for x in after}
        # neither task should be returned as they stop before `until_after`
        self.assertNotIn(task1["id"], tasks)
        self.assertNotIn(task2["id"], tasks)

        resp = self.client.get(f"/scheduled_tasks/{task1['id']}")
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks/{task2['id']}")
        self.assertEqual(200, resp.status_code)

        resp = self.client.delete(f"/scheduled_tasks/{task1['id']}")
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks/{task1['id']}")
        self.assertEqual(404, resp.status_code)
        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until}"
        )
        tasks = {x["id"]: x for x in resp.json()}
        self.assertNotIn(task1["id"], tasks)
        # task 2 should not be deleted
        self.assertIn(task2["id"], tasks)

        resp = self.client.delete(f"/scheduled_tasks/{task2['id']}")
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks/{task2['id']}")
        self.assertEqual(404, resp.status_code)
        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until}"
        )
        tasks = {x["id"]: x for x in resp.json()}
        self.assertNotIn(task1["id"], tasks)
        self.assertNotIn(task2["id"], tasks)

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
            "schedules": [
                {
                    "period": "day",
                }
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task)
        self.assertEqual(201, resp.status_code)
        task = resp.json()

        resp = self.client.get("/scheduled_tasks?start_before=0&until_after=0")
        self.assertEqual(200, resp.status_code, resp.json())
        tasks = {x["id"]: x for x in resp.json()}
        self.assertIn(task["id"], tasks)

    def test_delete_scheduled_task(self):
        schedule_date_str = "2024-02-10T10:00:00+08:00"
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
        # testing server operates in GMT+8, while all the dates are
        # transmitted to the server using UTC.

        # 0800 GMT+8 10th February 2024, is 0000 UTC 10th February 2024
        scheduled_date_str = "2024-02-10T00:00:00+00:00"
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

        # delete a single event from this schedule, for 0800 GMT+8 11th February 2024,
        # which is 0000 UTC 11th February 2024.
        schedule_task_id = scheduled_task["id"]
        params = {"event_date": "2024-02-11T00:00:00+00:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # the except_date should be 11th February 2024
        self.assertEqual(len(scheduled_task["except_dates"]), 1, scheduled_task)
        self.assertEqual(
            scheduled_task["except_dates"][0], "2024-02-11", scheduled_task
        )

        # delete a single event from this schedule, for 0800 GMT+8 12th February 2024
        schedule_task_id = scheduled_task["id"]
        params = {"event_date": "2024-02-12T08:00:00+08:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # the except_date should now have both 11th and 12th February 2024
        self.assertEqual(len(scheduled_task["except_dates"]), 2, scheduled_task)
        self.assertTrue("2024-02-11" in scheduled_task["except_dates"], scheduled_task)
        self.assertTrue("2024-02-12" in scheduled_task["except_dates"], scheduled_task)

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")

    def test_delete_scheduled_task_event_after_eight_am(self):
        # testing server operates in GMT+8, while all the dates are
        # transmitted to the server using UTC.

        # 0801 GMT+8 10th February 2024, is 0001 UTC 10th February 2024
        scheduled_date_str = "2024-02-10T00:01:00+00:00"
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

        # delete a single event from this schedule, for 0801 GMT+8 11th February 2024,
        # which is 0001 UTC 11th February 2024.
        schedule_task_id = scheduled_task["id"]
        params = {"event_date": "2024-02-11T00:01:00+00:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # the except_date should be 11th February 2024
        self.assertEqual(len(scheduled_task["except_dates"]), 1, scheduled_task)
        self.assertEqual(
            scheduled_task["except_dates"][0], "2024-02-11", scheduled_task
        )

        # delete a single event from this schedule, for 0801 GMT+8 12th February 2024
        schedule_task_id = scheduled_task["id"]
        params = {"event_date": "2024-02-12T08:01:00+08:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # the except_date should now have both 11th and 12th February 2024
        self.assertEqual(len(scheduled_task["except_dates"]), 2, scheduled_task)
        self.assertTrue("2024-02-11" in scheduled_task["except_dates"], scheduled_task)
        self.assertTrue("2024-02-12" in scheduled_task["except_dates"], scheduled_task)

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")

    def test_delete_scheduled_task_event_before_eight_am(self):
        # testing server operates in GMT+8, all the dates are transmitted
        # to the server using UTC.

        # every day at 0759 GMT+8 starting from 10th February 2024,
        # which is 2359 UTC 9th February 2024.
        scheduled_date_str = "2024-02-09T23:59:00+00:00"
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

        # delete a single event from this schedule, for 0759 GMT+8 11th February 2024,
        # which is 2359 UTC 10th February 2024.
        schedule_task_id = scheduled_task["id"]
        params = {"event_date": "2024-02-10T23:59:00+00:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # since the server is operating in GMT+8, the except_date should be
        # 11th February 2024, and not 10th February 2024
        self.assertEqual(len(scheduled_task["except_dates"]), 1, scheduled_task)
        self.assertEqual(
            scheduled_task["except_dates"][0], "2024-02-11", scheduled_task
        )

        # delete a single event from this schedule, for 0759 GMT+8 12th February 2024
        schedule_task_id = scheduled_task["id"]
        params = {"event_date": "2024-02-12T07:59:00+08:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # since the server is operating in GMT+8, the except_date should have
        # both 11th and 12th February 2024
        self.assertEqual(len(scheduled_task["except_dates"]), 2, scheduled_task)
        self.assertTrue("2024-02-11" in scheduled_task["except_dates"], scheduled_task)
        self.assertTrue("2024-02-12" in scheduled_task["except_dates"], scheduled_task)

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")

    def test_delete_scheduled_task_event_at_local_midnight(self):
        # testing server operates in GMT+8, all the dates are transmitted
        # to the server using UTC.

        # every day at 0000 GMT+8 starting from 12th February 2024,
        # which is 1600 UTC 11th February 2024.
        scheduled_date_str = "2024-02-11T16:00:00+00:00"
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

        # delete a single event from this schedule, for 0000 GMT+8 13th February 2024,
        # which is 1600 UTC 12th February 2024.
        schedule_task_id = scheduled_task["id"]
        params = {"event_date": "2024-02-12T16:00:00+00:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # since the server is operating in GMT+8, the except_date should be
        # 13th February 2024, and not 12th February 2024
        self.assertEqual(len(scheduled_task["except_dates"]), 1, scheduled_task)
        self.assertEqual(
            scheduled_task["except_dates"][0], "2024-02-13", scheduled_task
        )

        # delete a single event from this schedule, for 0000 GMT+8 14th February 2024
        schedule_task_id = scheduled_task["id"]
        params = {"event_date": "2024-02-14T00:00:00+08:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

        # check the same scheduled task
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)

        # The except_date should be both 13th and 14th February 2024
        self.assertEqual(len(scheduled_task["except_dates"]), 2, scheduled_task)
        self.assertTrue("2024-02-13" in scheduled_task["except_dates"], scheduled_task)
        self.assertTrue("2024-02-14" in scheduled_task["except_dates"], scheduled_task)

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")
