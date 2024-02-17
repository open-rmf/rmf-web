# pylint: disable=too-many-lines
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
        params = {"event_date": "2124-02-11T00:00:00+00:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

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
        params = {"event_date": "2124-02-12T08:00:00+08:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

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
        params = {"event_date": "2124-02-11T00:01:00+00:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

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
        params = {"event_date": "2124-02-12T08:01:00+08:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

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
        params = {"event_date": "2124-02-10T23:59:00+00:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

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
        params = {"event_date": "2124-02-12T07:59:00+08:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

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
        params = {"event_date": "2124-02-12T16:00:00+00:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

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
        params = {"event_date": "2124-02-14T00:00:00+08:00"}
        self.client.put(
            f"/scheduled_tasks/{schedule_task_id}/clear?{urlencode(params)}"
        )

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
        scheduled_date_str = "2124-02-10T16:00:00+08:00"
        task_request = {
            "category": "test_category",
            "description": "test_description",
        }
        task_schedules = [
            {
                "period": "day",
                "start_from": scheduled_date_str,
                "at": "16:00",
            }
        ]

        scheduled_task_description = {
            "task_request": task_request,
            "schedules": task_schedules,
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)
        schedule_task_id = scheduled_task["id"]
        schedules = scheduled_task["schedules"]

        # update task description
        updated_task_request = {
            "category": "test_category_updated",
            "description": "test_description_updated",
        }
        scheduled_task_description["task_request"] = updated_task_request
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update",
            json=scheduled_task_description,
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # compare updated scheduled task description
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"],
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertEqual(
            updated_scheduled_task["schedules"], schedules, updated_scheduled_task
        )

        # update task schedules in UTC
        updated_scheduled_date_str = "2124-02-10T12:00:00+00:00"
        updated_task_schedules = [
            {
                "period": "day",
                "start_from": updated_scheduled_date_str,
                "at": "20:00",
            }
        ]
        scheduled_task_description["schedules"] = updated_task_schedules
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update",
            json=scheduled_task_description,
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # compare updated scheduled task schedules
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"],
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertNotEqual(
            updated_scheduled_task["schedules"], schedules, updated_scheduled_task
        )
        self.assertEqual(
            len(updated_scheduled_task["schedules"]), 1, updated_scheduled_task
        )
        updated_schedule = updated_scheduled_task["schedules"][0]
        self.assertEqual(
            updated_schedule["start_from"], updated_scheduled_date_str, updated_schedule
        )
        self.assertEqual(updated_schedule["at"], "20:00", updated_schedule)

        # update task schedules in GMT+8
        updated_scheduled_date_str = "2124-02-10T21:00:00+08:00"
        updated_task_schedules = [
            {
                "period": "day",
                "start_from": updated_scheduled_date_str,
                "at": "21:00",
            }
        ]
        scheduled_task_description["schedules"] = updated_task_schedules
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update",
            json=scheduled_task_description,
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # compare updated scheduled task schedules
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"],
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertNotEqual(
            updated_scheduled_task["schedules"], schedules, updated_scheduled_task
        )
        self.assertEqual(
            len(updated_scheduled_task["schedules"]), 1, updated_scheduled_task
        )
        updated_schedule = updated_scheduled_task["schedules"][0]

        # Start from dates are saved as UTC
        updated_scheduled_date_utc_str = "2124-02-10T13:00:00+00:00"
        self.assertEqual(
            updated_schedule["start_from"],
            updated_scheduled_date_utc_str,
            updated_schedule,
        )
        self.assertEqual(updated_schedule["at"], "21:00", updated_schedule)

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")

    def test_edit_scheduled_task_event(self):
        scheduled_date_str = "2124-02-10T10:00:00+08:00"
        task_request = {
            "category": "test_category",
            "description": "test_description",
        }
        task_schedules = [
            {
                "period": "day",
                "start_from": scheduled_date_str,
                "at": "10:00",
            }
        ]

        scheduled_task_description = {
            "task_request": task_request,
            "schedules": task_schedules,
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)
        schedule_task_id = scheduled_task["id"]
        schedules = scheduled_task["schedules"]

        # update task description, with except date on 12th GMT+8
        except_scheduled_date_str = "2124-02-12T10:00:00+08:00"
        updated_task_request = {
            "category": "test_category_updated",
            "description": "test_description_updated",
        }
        scheduled_task_description["task_request"] = updated_task_request

        # this will be accompanied by an until date of 23:59 12th GMT+8, which
        # will be saved as 15:59 12th UTC
        # FIXME this should come in as client dashboard's time zone
        until_str_utc = "2124-02-12T15:59:59+00:00"
        updated_task_schedule = task_schedules[0]
        updated_task_schedule["until"] = until_str_utc
        scheduled_task_description["schedules"][0] = updated_task_schedule

        params = {"except_date": except_scheduled_date_str}
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update?{urlencode(params)}",
            json=scheduled_task_description,
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # since we have an except_date, the original schedule should not change
        # except an additional except_date
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"], task_request, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["schedules"], schedules, updated_scheduled_task
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 1, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["except_dates"][0],
            "2124-02-12",
            updated_scheduled_task,
        )

        # The new schedule task for that single event should just be a single index increment
        # This scheduled task with a single event should have the
        # - updated task description
        # - no except dates
        # - correct start_from date
        # - correct until date in UTC
        new_scheduled_task_id = schedule_task_id + 1
        resp = self.client.get(f"/scheduled_tasks/{new_scheduled_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"],
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 0, updated_scheduled_task
        )
        single_event_schedules = updated_scheduled_task["schedules"]
        self.assertEqual(len(single_event_schedules), 1, single_event_schedules)
        self.assertEqual(
            single_event_schedules[0]["at"],
            updated_task_schedule["at"],
            single_event_schedules,
        )
        self.assertEqual(
            single_event_schedules[0]["period"],
            updated_task_schedule["period"],
            single_event_schedules,
        )
        self.assertIsNotNone(single_event_schedules[0]["until"])
        # until date is saved in UTC
        self.assertEqual(
            single_event_schedules[0]["until"], until_str_utc, single_event_schedules[0]
        )

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")
        self.client.delete(f"/scheduled_tasks/{new_scheduled_task_id}")

    def test_edit_scheduled_task_event_at_eight_am(self):
        scheduled_date_str = "2124-02-10T08:00:00+08:00"
        task_request = {
            "category": "test_category",
            "description": "test_description",
        }
        task_schedules = [
            {
                "period": "day",
                "start_from": scheduled_date_str,
                "at": "08:00",
            }
        ]

        scheduled_task_description = {
            "task_request": task_request,
            "schedules": task_schedules,
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)
        schedule_task_id = scheduled_task["id"]
        schedules = scheduled_task["schedules"]

        # update task description, with except date on 12th GMT+8
        except_scheduled_date_str = "2124-02-12T08:00:00+08:00"
        updated_task_request = {
            "category": "test_category_updated",
            "description": "test_description_updated",
        }
        scheduled_task_description["task_request"] = updated_task_request

        # this will be accompanied by an until date of 23:59 12th GMT+8, which
        # will be saved as 15:59 12th UTC
        # FIXME this should come in as client dashboard's time zone
        until_str_utc = "2124-02-12T15:59:59+00:00"
        updated_task_schedule = task_schedules[0]
        updated_task_schedule["until"] = until_str_utc
        scheduled_task_description["schedules"][0] = updated_task_schedule

        params = {"except_date": except_scheduled_date_str}
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update?{urlencode(params)}",
            json=scheduled_task_description,
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # since we have an except_date, the original schedule should not change
        # except an additional except_date
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"], task_request, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["schedules"], schedules, updated_scheduled_task
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 1, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["except_dates"][0],
            "2124-02-12",
            updated_scheduled_task,
        )

        # The new schedule task for that single event should just be a single index increment
        # This scheduled task with a single event should have the
        # - updated task description
        # - no except dates
        # - correct start_from date
        # - correct until date
        new_scheduled_task_id = schedule_task_id + 1
        resp = self.client.get(f"/scheduled_tasks/{new_scheduled_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"],
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 0, updated_scheduled_task
        )
        single_event_schedules = updated_scheduled_task["schedules"]
        self.assertEqual(len(single_event_schedules), 1, single_event_schedules)
        self.assertEqual(
            single_event_schedules[0]["at"],
            updated_task_schedule["at"],
            single_event_schedules,
        )
        self.assertEqual(
            single_event_schedules[0]["period"],
            updated_task_schedule["period"],
            single_event_schedules,
        )
        self.assertIsNotNone(single_event_schedules[0]["until"])
        # until date is saved in UTC
        self.assertEqual(
            single_event_schedules[0]["until"], until_str_utc, single_event_schedules[0]
        )

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")
        self.client.delete(f"/scheduled_tasks/{new_scheduled_task_id}")

    def test_edit_scheduled_task_event_after_eight_am(self):
        scheduled_date_str = "2124-02-11T09:00:00+08:00"
        task_request = {
            "category": "test_category",
            "description": "test_description",
        }
        task_schedules = [
            {
                "period": "day",
                "start_from": scheduled_date_str,
                "at": "09:00",
            }
        ]

        scheduled_task_description = {
            "task_request": task_request,
            "schedules": task_schedules,
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)
        schedule_task_id = scheduled_task["id"]
        schedules = scheduled_task["schedules"]

        # update task description, with except date on 14th GMT+8
        except_scheduled_date_str = "2124-02-14T09:00:00+08:00"
        updated_task_request = {
            "category": "test_category_updated",
            "description": "test_description_updated",
        }
        scheduled_task_description["task_request"] = updated_task_request

        # this will be accompanied by an until date of 23:59 14th GMT+8, which
        # will be saved as 15:59 14th UTC
        # FIXME this should come in as client dashboard's time zone
        until_str_utc = "2124-02-14T15:59:59+00:00"
        updated_task_schedule = task_schedules[0]
        updated_task_schedule["until"] = until_str_utc
        scheduled_task_description["schedules"][0] = updated_task_schedule

        params = {"except_date": except_scheduled_date_str}
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update?{urlencode(params)}",
            json=scheduled_task_description,
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # since we have an except_date, the original schedule should not change
        # except an additional except_date
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"], task_request, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["schedules"], schedules, updated_scheduled_task
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 1, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["except_dates"][0],
            "2124-02-14",
            updated_scheduled_task,
        )

        # The new schedule task for that single event should just be a single index increment
        # This scheduled task with a single event should have the
        # - updated task description
        # - no except dates
        # - correct start_from date
        # - correct until date in UTC
        new_scheduled_task_id = schedule_task_id + 1
        resp = self.client.get(f"/scheduled_tasks/{new_scheduled_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"],
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 0, updated_scheduled_task
        )
        single_event_schedules = updated_scheduled_task["schedules"]
        self.assertEqual(len(single_event_schedules), 1, single_event_schedules)
        self.assertEqual(
            single_event_schedules[0]["at"],
            updated_task_schedule["at"],
            single_event_schedules,
        )
        self.assertEqual(
            single_event_schedules[0]["period"],
            updated_task_schedule["period"],
            single_event_schedules,
        )
        self.assertIsNotNone(single_event_schedules[0]["until"])
        # until date is saved in UTC
        self.assertEqual(
            single_event_schedules[0]["until"], until_str_utc, single_event_schedules[0]
        )

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")
        self.client.delete(f"/scheduled_tasks/{new_scheduled_task_id}")

    def test_edit_scheduled_task_event_before_eight_am(self):
        scheduled_date_str = "2124-02-16T07:00:00+08:00"
        task_request = {
            "category": "test_category",
            "description": "test_description",
        }
        task_schedules = [
            {
                "period": "day",
                "start_from": scheduled_date_str,
                "at": "07:00",
            }
        ]

        scheduled_task_description = {
            "task_request": task_request,
            "schedules": task_schedules,
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)
        schedule_task_id = scheduled_task["id"]
        schedules = scheduled_task["schedules"]

        # update task description, with except date on 18th GMT+8
        except_scheduled_date_str = "2124-02-18T07:00:00+08:00"
        updated_task_request = {
            "category": "test_category_updated",
            "description": "test_description_updated",
        }
        scheduled_task_description["task_request"] = updated_task_request

        # this will be accompanied by an until date of 23:59 18th GMT+8, which
        # will be saved as 15:59 18th UTC
        # FIXME this should come in as client dashboard's time zone
        until_str_utc = "2124-02-18T15:59:59+00:00"
        updated_task_schedule = task_schedules[0]
        updated_task_schedule["until"] = until_str_utc
        scheduled_task_description["schedules"][0] = updated_task_schedule

        params = {"except_date": except_scheduled_date_str}
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update?{urlencode(params)}",
            json=scheduled_task_description,
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # since we have an except_date, the original schedule should not change
        # except an additional except_date
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"], task_request, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["schedules"], schedules, updated_scheduled_task
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 1, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["except_dates"][0],
            "2124-02-18",
            updated_scheduled_task,
        )

        # The new schedule task for that single event should just be a single index increment
        # This scheduled task with a single event should have the
        # - updated task description
        # - no except dates
        # - correct start_from date
        # - correct until date in UTC
        new_scheduled_task_id = schedule_task_id + 1
        resp = self.client.get(f"/scheduled_tasks/{new_scheduled_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"],
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 0, updated_scheduled_task
        )
        single_event_schedules = updated_scheduled_task["schedules"]
        self.assertEqual(len(single_event_schedules), 1, single_event_schedules)
        self.assertEqual(
            single_event_schedules[0]["at"],
            updated_task_schedule["at"],
            single_event_schedules,
        )
        self.assertEqual(
            single_event_schedules[0]["period"],
            updated_task_schedule["period"],
            single_event_schedules,
        )
        self.assertIsNotNone(single_event_schedules[0]["until"])
        # until date is saved in UTC
        self.assertEqual(
            single_event_schedules[0]["until"], until_str_utc, single_event_schedules[0]
        )

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")
        self.client.delete(f"/scheduled_tasks/{new_scheduled_task_id}")

    def test_edit_scheduled_task_event_at_local_midnight(self):
        scheduled_date_str = "2124-02-16T00:00:00+08:00"
        task_request = {
            "category": "test_category",
            "description": "test_description",
        }
        task_schedules = [
            {
                "period": "day",
                "start_from": scheduled_date_str,
                "at": "00:00",
            }
        ]

        scheduled_task_description = {
            "task_request": task_request,
            "schedules": task_schedules,
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)
        schedule_task_id = scheduled_task["id"]
        schedules = scheduled_task["schedules"]

        # update task description, with except date on 18th GMT+8
        except_scheduled_date_str = "2124-02-18T00:00:00+08:00"
        updated_task_request = {
            "category": "test_category_updated",
            "description": "test_description_updated",
        }
        scheduled_task_description["task_request"] = updated_task_request

        # this will be accompanied by an until date of 23:59 18th GMT+8, which
        # will be saved as 15:59 18th UTC
        # FIXME this should come in as client dashboard's time zone
        until_str_utc = "2124-02-18T15:59:59+00:00"
        updated_task_schedule = task_schedules[0]
        updated_task_schedule["until"] = until_str_utc
        scheduled_task_description["schedules"][0] = updated_task_schedule

        params = {"except_date": except_scheduled_date_str}
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update?{urlencode(params)}",
            json=scheduled_task_description,
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # since we have an except_date, the original schedule should not change
        # except an additional except_date
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"], task_request, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["schedules"], schedules, updated_scheduled_task
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 1, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["except_dates"][0],
            "2124-02-18",
            updated_scheduled_task,
        )

        # The new schedule task for that single event should just be a single index increment
        # This scheduled task with a single event should have the
        # - updated task description
        # - no except dates
        # - correct start_from date
        # - correct until date in UTC
        new_scheduled_task_id = schedule_task_id + 1
        resp = self.client.get(f"/scheduled_tasks/{new_scheduled_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"],
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 0, updated_scheduled_task
        )
        single_event_schedules = updated_scheduled_task["schedules"]
        self.assertEqual(len(single_event_schedules), 1, single_event_schedules)
        self.assertEqual(
            single_event_schedules[0]["at"],
            updated_task_schedule["at"],
            single_event_schedules,
        )
        self.assertEqual(
            single_event_schedules[0]["period"],
            updated_task_schedule["period"],
            single_event_schedules,
        )
        self.assertIsNotNone(single_event_schedules[0]["until"])
        # until date is saved in UTC
        self.assertEqual(
            single_event_schedules[0]["until"], until_str_utc, single_event_schedules[0]
        )

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")
        self.client.delete(f"/scheduled_tasks/{new_scheduled_task_id}")

    def test_edit_scheduled_task_after_edit_scheduled_task_event(self):
        scheduled_date_str = "2124-02-11T09:00:00+08:00"
        task_request = {
            "category": "test_category",
            "description": "test_description",
        }
        task_schedules = [
            {
                "period": "day",
                "start_from": scheduled_date_str,
                "at": "09:00",
            }
        ]

        scheduled_task_description = {
            "task_request": task_request,
            "schedules": task_schedules,
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_description)
        self.assertEqual(201, resp.status_code, resp.json())
        scheduled_task = resp.json()
        self.assertEqual(len(scheduled_task["schedules"]), 1, scheduled_task)
        schedule_task_id = scheduled_task["id"]
        schedules = scheduled_task["schedules"]

        # update task description, with except date on 14th GMT+8
        except_scheduled_date_str = "2124-02-14T09:00:00+08:00"
        updated_task_request = {
            "category": "test_category_updated",
            "description": "test_description_updated",
        }
        scheduled_task_description["task_request"] = updated_task_request

        # this will be accompanied by an until date of 23:59 14th GMT+8, which
        # will be saved as 15:59 14th UTC
        # FIXME this should come in as client dashboard's time zone
        until_str_utc = "2124-02-14T15:59:59+00:00"
        updated_task_schedule = task_schedules[0]
        updated_task_schedule["until"] = until_str_utc
        scheduled_task_description["schedules"][0] = updated_task_schedule

        params = {"except_date": except_scheduled_date_str}
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update?{urlencode(params)}",
            json=scheduled_task_description,
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # since we have an except_date, the original schedule should not change
        # except an additional except_date
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"], task_request, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["schedules"], schedules, updated_scheduled_task
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 1, updated_scheduled_task
        )
        self.assertEqual(
            updated_scheduled_task["except_dates"][0],
            "2124-02-14",
            updated_scheduled_task,
        )

        # The new schedule task for that single event should just be a single index increment
        # This scheduled task with a single event should have the
        # - updated task description
        # - no except dates
        # - correct start_from date
        # - correct until date in UTC
        new_scheduled_task_id = schedule_task_id + 1
        resp = self.client.get(f"/scheduled_tasks/{new_scheduled_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"],
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 0, updated_scheduled_task
        )
        single_event_schedules = updated_scheduled_task["schedules"]
        self.assertEqual(len(single_event_schedules), 1, single_event_schedules)
        self.assertEqual(
            single_event_schedules[0]["at"],
            updated_task_schedule["at"],
            single_event_schedules,
        )
        self.assertEqual(
            single_event_schedules[0]["period"],
            updated_task_schedule["period"],
            single_event_schedules,
        )
        self.assertIsNotNone(single_event_schedules[0]["until"])
        # until date is saved in UTC
        self.assertEqual(
            single_event_schedules[0]["until"], until_str_utc, single_event_schedules[0]
        )

        # update original task schedules in GMT+8
        updated_scheduled_date_str = "2124-02-11T14:00:00+08:00"
        updated_task_schedules = [
            {
                "period": "day",
                "start_from": updated_scheduled_date_str,
                "at": "14:00",
            }
        ]
        scheduled_task_description["schedules"] = updated_task_schedules
        resp = self.client.post(
            f"/scheduled_tasks/{schedule_task_id}/update",
            json=scheduled_task_description,
        )
        self.assertEqual(201, resp.status_code, resp.json())

        # compare updated scheduled task schedules
        resp = self.client.get(f"/scheduled_tasks/{schedule_task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        updated_scheduled_task = resp.json()
        self.assertEqual(
            updated_scheduled_task["task_request"],
            updated_task_request,
            updated_scheduled_task,
        )
        self.assertNotEqual(
            updated_scheduled_task["schedules"], schedules, updated_scheduled_task
        )
        self.assertEqual(
            len(updated_scheduled_task["schedules"]), 1, updated_scheduled_task
        )
        updated_schedule = updated_scheduled_task["schedules"][0]
        # when editing an entire schedule, the exempted dates are ignored, so it will be empty
        self.assertEqual(
            len(updated_scheduled_task["except_dates"]), 0, updated_scheduled_task
        )

        # FIXME Start from dates are saved as UTC
        updated_scheduled_date_utc_str = "2124-02-11T06:00:00+00:00"
        self.assertEqual(
            updated_schedule["start_from"],
            updated_scheduled_date_utc_str,
            updated_schedule,
        )
        self.assertEqual(updated_schedule["at"], "14:00", updated_schedule)

        # cleanup
        self.client.delete(f"/scheduled_tasks/{schedule_task_id}")
        self.client.delete(f"/scheduled_tasks/{new_scheduled_task_id}")
