from unittest.mock import patch
from uuid import uuid4

from api_server import models as mdl
from api_server.repositories import TaskRepository
from api_server.rmf_io import task_events, tasks_service
from api_server.test import AppFixture, make_task_log, make_task_state, test_user


class TestTasksRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        task_ids = [uuid4()]
        cls.task_states = [make_task_state(task_id=f"test_{x}") for x in task_ids]
        cls.task_logs = [make_task_log(task_id=f"test_{x}") for x in task_ids]
        repo = TaskRepository(test_user)

        async def prepare_db():
            for t in cls.task_states:
                await repo.save_task_state(t)
            for t in cls.task_logs:
                await repo.save_task_log(t)

        cls.run_in_app_loop(prepare_db())

    def test_get_task_state(self):
        resp = self.session.get(f"/tasks/{self.task_states[0].booking.id}/state")
        self.assertEqual(200, resp.status_code)
        self.assertEqual(
            self.task_states[0].booking.id,
            resp.json()["booking"]["id"],
        )

    def test_query_task_states(self):
        resp = self.session.get(f"/tasks?task_id={self.task_states[0].booking.id}")
        self.assertEqual(200, resp.status_code)
        results = resp.json()
        self.assertEqual(1, len(results))
        self.assertEqual(self.task_states[0].booking.id, results[0]["booking"]["id"])

    def test_sub_task_state(self):
        fut = self.subscribe_sio(f"/tasks/{self.task_states[0].booking.id}/state")
        result = fut.result(1)
        self.assertEqual(self.task_states[0].booking.id, result["booking"]["id"])

    def test_get_task_log(self):
        resp = self.session.get(
            f"/tasks/{self.task_logs[0].task_id}/log?between=0,1636388414500"
        )
        self.assertEqual(200, resp.status_code)
        logs = mdl.TaskEventLog(**resp.json())
        self.assertEqual(self.task_logs[0].task_id, logs.task_id)

        # check task log
        if logs.log is None:
            self.assertIsNotNone(logs.log)
            return
        self.assertEqual(1, len(logs.log))
        log = logs.log[0]
        self.assertEqual(0, log.seq)
        self.assertEqual(mdl.Tier.info, log.tier)
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
        phase1_log = phase1.log
        if phase1_log is None:
            self.assertIsNotNone(phase1_log)
            return
        self.assertEqual(1, len(phase1_log))
        log = phase1_log[0]
        self.assertEqual(0, log.seq)
        self.assertEqual(mdl.Tier.info, log.tier)
        self.assertEqual(1636388410000, log.unix_millis_time)
        self.assertEqual("Beginning phase", log.text)

        # check number of events
        phase1_events = phase1.events
        if phase1_events is None:
            self.assertIsNotNone(phase1_events)
            return
        self.assertEqual(
            7, len(phase1_events)
        )  # check all events are returned, including those with no logs

        # check event log
        self.assertIn("1", phase1_events)
        self.assertEqual(
            3, len(phase1_events["1"])
        )  # check only logs in the period is returned
        log = phase1_events["1"][0]
        self.assertEqual(0, log.seq)
        self.assertEqual(mdl.Tier.info, log.tier)
        self.assertEqual(1636388409995, log.unix_millis_time)
        self.assertEqual(
            "Generating plan to get from [place:parking_03] to [place:kitchen]",
            log.text,
        )

        # TODO: check relative time is working, this requires the use of
        # dependencies overrides, which requires change in the server architecture.
        # Better to do this after this gets merged into main so we don't make
        # more architecture changes.

    def test_sub_task_log(self):
        task_id = f"task_{uuid4()}"
        fut = self.subscribe_sio(f"/tasks/{task_id}/log")
        task_logs = make_task_log(task_id)
        task_events.task_event_logs.on_next(task_logs)
        result = fut.result(1)
        self.assertEqual(task_id, result["task_id"])

    def test_activity_discovery(self):
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = "{}"
            resp = self.session.post(
                "/tasks/activity_discovery",
                data=mdl.ActivityDiscoveryRequest(
                    type="activitiy_discovery_request"
                ).json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_cancel_task(self):
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = '{ "success": true }'
            resp = self.session.post(
                "/tasks/activity_discovery",
                data=mdl.ActivityDiscoveryRequest(
                    type="activitiy_discovery_request"
                ).json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_interrupt_task(self):
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = '{ "success": True, "token": "token" }'
            resp = self.session.post(
                "/tasks/interrupt_task",
                data=mdl.TaskInterruptionRequest(  # type: ignore
                    type="interrupt_task_request", task_id="task_id"
                ).json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_kill_task(self):
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = '{ "success": true }'
            resp = self.session.post(
                "/tasks/kill_task",
                data=mdl.TaskKillRequest(  # type: ignore
                    type="kill_task_request", task_id="task_id"
                ).json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_resume_task(self):
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = '{ "success": true }'
            resp = self.session.post(
                "/tasks/resume_task",
                data=mdl.TaskResumeRequest().json(exclude_none=True),  # type: ignore
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_rewind_task(self):
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = '{ "success": true }'
            resp = self.session.post(
                "/tasks/rewind_task",
                data=mdl.TaskRewindRequest(
                    type="rewind_task_request", task_id="task_id", phase_id=0
                ).json(
                    exclude_none=True
                ),  # type: ignore
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_skip_phase(self):
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = '{ "success": True, "token": "token" }'
            resp = self.session.post(
                "/tasks/skip_phase",
                data=mdl.TaskPhaseSkipRequest(  # type: ignore
                    type="skip_phase_request", task_id="task_id", phase_id=0
                ).json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_task_discovery(self):
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = "{}"
            resp = self.session.post(
                "/tasks/task_discovery",
                data=mdl.TaskDiscoveryRequest(type="task_discovery_request").json(
                    exclude_none=True
                ),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_undo_skip_phase(self):
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = '{ "success": True }'
            resp = self.session.post(
                "/tasks/undo_skip_phase",
                data=mdl.UndoPhaseSkipRequest(type="undo_phase_skip_request").json(exclude_none=True),  # type: ignore
            )
            self.assertEqual(200, resp.status_code, resp.content)


class TestDispatchTask(AppFixture):
    def post_task_request(self):
        return self.session.post(
            "/tasks/dispatch_task",
            data=mdl.DispatchTaskRequest(
                type="dispatch_task_request",
                request=mdl.TaskRequest(
                    category="test",
                    description="description",
                ),  # type: ignore
            ).json(exclude_none=True),
        )

    def test_success(self):
        task_id = str(uuid4())
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = f'{{ "success": true, "state": {{ "booking": {{ "id": "{task_id}" }} }} }}'
            resp = self.post_task_request()
            self.assertEqual(200, resp.status_code, resp.content)

        # check that the task is already in the database by the time the dispatch request returns
        resp = self.session.get(f"/tasks/{task_id}/state")
        self.assertEqual(200, resp.status_code, resp.content)
        self.assertEqual(task_id, resp.json()["booking"]["id"])

    def test_fail_with_multiple_errors(self):
        # fails with multiple errors
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = """{
                "success": false,
                "errors": [
                    { "code": 1, "category": "test_error_1", "detail": "detail 1" },
                    { "code": 2, "category": "test_error_2", "detail": "detail 2" }
                ]
            }
            """
            resp = self.post_task_request()
            self.assertEqual(400, resp.status_code, resp.content)

    def test_fail_with_no_errors(self):
        # fails with multiple errors
        with patch.object(tasks_service, "call") as mock:
            mock.return_value = """{
                "success": false
            }
            """
            resp = self.post_task_request()
            self.assertEqual(400, resp.status_code, resp.content)
