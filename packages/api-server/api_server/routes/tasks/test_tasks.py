from uuid import uuid4

from api_server.models import TaskEventLog
from api_server.test import AppFixture, make_task_log, make_task_state, try_until


class TestTasksRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        task_ids = [uuid4()]
        cls.task_states = [make_task_state(task_id=f"test_{x}") for x in task_ids]
        cls.task_logs = [make_task_log(task_id=f"test_{x}") for x in task_ids]

        async def prepare_db():
            for t in cls.task_states:
                await t.save()
            for t in cls.task_logs:
                await t.save()

        cls.run_in_app_loop(prepare_db())

    def test_get_task_state(self):
        resp = self.session.get(f"/tasks/{self.task_states[0].booking.id}/state")
        self.assertEqual(200, resp.status_code)
        self.assertEqual(self.task_states[0].booking.id, resp.json()["booking"]["id"])

    def test_query_task_states(self):
        resp = self.session.get(f"/tasks?task_id={self.task_states[0].booking.id}")
        self.assertEqual(200, resp.status_code)
        results = resp.json()
        self.assertEqual(1, len(results))
        self.assertEqual(self.task_states[0].booking.id, results[0]["booking"]["id"])

    def test_sub_task_state(self):
        fut = self.subscribe_sio(f"/tasks/{self.task_states[0].booking.id}/state")
        try_until(fut.done, lambda x: x)
        result = fut.result(0)
        self.assertEqual(self.task_states[0].booking.id, result["booking"]["id"])

    def test_get_task_log(self):
        resp = self.session.get(
            f"/tasks/{self.task_logs[0].task_id}/log?between=0,1636388414500"
        )
        self.assertEqual(200, resp.status_code)
        logs = TaskEventLog(**resp.json())
        self.assertEqual(self.task_logs[0].task_id, logs.task_id)

        # check task log
        if logs.log is None:
            self.assertIsNotNone(logs.log)
            return
        self.assertEqual(1, len(logs.log))
        log = logs.log[0]
        self.assertEqual(0, log.seq)
        self.assertEqual("info", log.tier.name)
        self.assertEqual(1636388410000, log.unix_millis_time)
        self.assertEqual("Beginning task", log.text)

        # check number of phases
        if logs.phases is None:
            self.assertIsNotNone(logs.phases)
            return
        self.assertEqual(2, len(logs.phases))
        self.assertIn("1", logs.phases)
        self.assertIn("2", logs.phases)

        # check correct log
        phase1 = logs.phases["1"]
        self.assertIn("log", phase1)
        phase1_log = phase1["log"]
        self.assertEqual(1, len(phase1_log))
        log = phase1_log[0]
        self.assertEqual(0, log["seq"])
        self.assertEqual("info", log["tier"])
        self.assertEqual(1636388410000, log["unix_millis_time"])
        self.assertEqual("Beginning phase", log["text"])

        # check number of events
        self.assertIn("events", phase1)
        phase1_events = phase1["events"]
        self.assertEqual(
            3, len(phase1_events)
        )  # check only events with logs in the period are returned

        # check event log
        self.assertIn("1", phase1_events)
        self.assertEqual(
            3, len(phase1_events["1"])
        )  # check only logs in the period is returned
        log = phase1_events["1"][0]
        self.assertEqual(0, log["seq"])
        self.assertEqual("info", log["tier"])
        self.assertEqual(1636388409995, log["unix_millis_time"])
        self.assertEqual(
            "Generating plan to get from [place:parking_03] to [place:kitchen]",
            log["text"],
        )

        # TODO: check relative time is working, this requires the use of
        # dependencies overrides, which requires change in the server architecture.
        # Better to do this after this gets merged into main so we don't make
        # more architecture changes.

    def test_sub_task_log(self):
        fut = self.subscribe_sio(f"/tasks/{self.task_logs[0].task_id}/log")
        try_until(fut.done, lambda x: x)
        result = fut.result(0)
        self.assertEqual(self.task_logs[0].task_id, result["task_id"])
