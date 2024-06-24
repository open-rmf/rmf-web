from typing import cast
from unittest.mock import patch
from uuid import uuid4

import pydantic

from api_server import models as mdl
from api_server.models import TaskEventLog, TaskState
from api_server.repositories import TaskRepository
from api_server.rmf_io import task_events, tasks_service
from api_server.test import AppFixture, make_task_log, make_task_state


class TestTasksRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        task_ids = [str(uuid4()), str(uuid4())]
        cls.task_states = [
            make_task_state(
                task_id=task_ids[0],
                labels=[
                    "test_single",
                    "test_single_2=",
                    "test_kv=value",
                    "test_label_sort=zzz",
                    "test_label_sort_2=aaa",
                    "test_label_sort_3=bbb",
                ],
            ),
            make_task_state(
                task_id=task_ids[1],
                labels=["test_label_sort=aaa", "test_label_sort_3=bbb"],
            ),
        ]
        cls.task_logs = [make_task_log(task_id=f"test_{x}") for x in task_ids]
        cls.clsSetupErr: str | None = None

        portal = cls.get_portal()
        repo = TaskRepository(cls.admin_user)
        for x in cls.task_states:
            portal.call(repo.save_task_state, x)
        for x in cls.task_logs:
            portal.call(repo.save_task_log, x)

    def setUp(self):
        super().setUp()
        self.assertIsNone(self.clsSetupErr)

    def test_get_task_state(self):
        resp = self.client.get(f"/tasks/{self.task_states[0].booking.id}/state")
        self.assertEqual(200, resp.status_code)
        task_state = TaskState.model_validate_json(resp.content)
        self.assertEqual(
            self.task_states[0].booking.id,
            task_state.booking.id,
        )
        if task_state.booking.labels is None:
            self.fail("expected label not to be None")
        labels = mdl.Labels.from_strings(task_state.booking.labels)
        self.assertEqual("", labels.root["test_single"])
        self.assertEqual("", labels.root["test_single_2"])
        self.assertEqual("value", labels.root["test_kv"])

    def test_query_task_states(self):
        resp = self.client.get(f"/tasks?task_id={self.task_states[0].booking.id}")
        self.assertEqual(200, resp.status_code)
        results = resp.json()
        self.assertEqual(1, len(results))
        self.assertEqual(self.task_states[0].booking.id, results[0]["booking"]["id"])

    def test_query_task_states_filter_by_label(self):
        resp = self.client.get("/tasks?label=not_existing")
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(0, len(results))

        resp = self.client.get("/tasks?label=test_single")
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(1, len(results))
        self.assertEqual(self.task_states[0].booking.id, results[0].booking.id)

        resp = self.client.get("/tasks?label=test_single=wrong_value")
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(0, len(results))

        resp = self.client.get("/tasks?label=test_single_2=")
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(1, len(results))
        self.assertEqual(self.task_states[0].booking.id, results[0].booking.id)

        resp = self.client.get("/tasks?label=test_kv=value")
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(1, len(results))
        self.assertEqual(self.task_states[0].booking.id, results[0].booking.id)

        resp = self.client.get("/tasks?label=test_kv=wrong_value")
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(0, len(results))

        resp = self.client.get("/tasks?label=test_single,test_kv=value")
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(1, len(results))
        self.assertEqual(self.task_states[0].booking.id, results[0].booking.id)

        resp = self.client.get("/tasks?label=test_single,test_kv=wrong_value")
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(0, len(results))

    def test_query_task_states_sort_by_label(self):
        resp = self.client.get("/tasks?order_by=-label=test_label_sort")
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(2, len(results))
        for a, b in zip(self.task_states, results):
            self.assertEqual(a, b)

        resp = self.client.get("/tasks?order_by=label=test_label_sort")
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(2, len(results))
        for a, b in zip(self.task_states[::-1], results):
            self.assertEqual(a, b)

        # test sorting by multiple labels
        resp = self.client.get(
            "/tasks?order_by=label=test_label_sort,label=test_label_sort_3"
        )
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(2, len(results))
        for a, b in zip(self.task_states[::-1], results):
            self.assertEqual(a, b)

        # test that tasks without the label are not filtered out
        # we don't test the result order because different db has different behavior
        # of sorting NULL.
        resp = self.client.get("/tasks?order_by=label=test_label_sort_not_existing")
        self.assertEqual(200, resp.status_code)
        results = pydantic.TypeAdapter(list[TaskState]).validate_json(resp.content)
        self.assertEqual(2, len(results))

    def test_sub_task_state(self):
        task_id = self.task_states[0].booking.id
        with self.subscribe_sio(f"/tasks/{task_id}/state") as sub:
            task_events.task_states.on_next(self.task_states[0])
            state = TaskState(**next(sub))
            self.assertEqual(task_id, cast(TaskState, state).booking.id)

    def test_get_task_log(self):
        resp = self.client.get(
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
        log = logs.log[0]  # pylint: disable=unsubscriptable-object
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
        phase1 = logs.phases["1"]  # pylint: disable=unsubscriptable-object
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
        task_id = self.task_logs[0].task_id
        with self.subscribe_sio(f"/tasks/{task_id}/log") as sub:
            task_events.task_event_logs.on_next(self.task_logs[0])
            log = TaskEventLog(**next(sub))
            self.assertEqual(task_id, cast(TaskEventLog, log).task_id)

    def test_activity_discovery(self):
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = "{}"
            resp = self.client.post(
                "/tasks/activity_discovery",
                content=mdl.ActivityDiscoveryRequest(
                    type="activitiy_discovery_request",
                ).model_dump_json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_cancel_task(self):
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = '{ "success": true }'
            resp = self.client.post(
                "/tasks/activity_discovery",
                content=mdl.ActivityDiscoveryRequest(
                    type="activitiy_discovery_request"
                ).model_dump_json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_interrupt_task(self):
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = '{ "success": True, "token": "token" }'
            resp = self.client.post(
                "/tasks/interrupt_task",
                content=mdl.TaskInterruptionRequest(
                    type="interrupt_task_request", task_id="task_id", labels=None
                ).model_dump_json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_kill_task(self):
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = '{ "success": true }'
            resp = self.client.post(
                "/tasks/kill_task",
                content=mdl.TaskKillRequest(
                    type="kill_task_request", task_id="task_id", labels=None
                ).model_dump_json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_resume_task(self):
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = '{ "success": true }'
            resp = self.client.post(
                "/tasks/resume_task",
                content=mdl.TaskResumeRequest(
                    type=None, for_task=None, for_tokens=None, labels=None
                ).model_dump_json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_rewind_task(self):
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = '{ "success": true }'
            resp = self.client.post(
                "/tasks/rewind_task",
                content=mdl.TaskRewindRequest(
                    type="rewind_task_request", task_id="task_id", phase_id=0
                ).model_dump_json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_skip_phase(self):
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = '{ "success": True, "token": "token" }'
            resp = self.client.post(
                "/tasks/skip_phase",
                content=mdl.TaskPhaseSkipRequest(
                    type="skip_phase_request",
                    task_id="task_id",
                    phase_id=0,
                    labels=None,
                ).model_dump_json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_task_discovery(self):
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = "{}"
            resp = self.client.post(
                "/tasks/task_discovery",
                content=mdl.TaskDiscoveryRequest(
                    type="task_discovery_request"
                ).model_dump_json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)

    def test_undo_skip_phase(self):
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = '{ "success": True }'
            resp = self.client.post(
                "/tasks/undo_skip_phase",
                content=mdl.UndoPhaseSkipRequest(
                    type="undo_phase_skip_request",
                ).model_dump_json(exclude_none=True),
            )
            self.assertEqual(200, resp.status_code, resp.content)


class TestDispatchTask(AppFixture):
    def post_task_request(self):
        return self.client.post(
            "/tasks/dispatch_task",
            content=mdl.DispatchTaskRequest(
                type="dispatch_task_request",
                request=mdl.TaskRequest(
                    category="test",
                    description="description",
                ),
            ).model_dump_json(exclude_none=True),
        )

    def test_success(self):
        task_id = str(uuid4())
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = f'{{ "success": true, "state": {{ "booking": {{ "id": "{task_id}" }} }} }}'
            resp = self.post_task_request()
            self.assertEqual(200, resp.status_code, resp.content)

        # check that the task is already in the database by the time the dispatch request returns
        resp = self.client.get(f"/tasks/{task_id}/state")
        self.assertEqual(200, resp.status_code, resp.content)
        self.assertEqual(task_id, resp.json()["booking"]["id"])

    def test_task_request_exist(self):
        task_id = str(uuid4())
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = f'{{ "success": true, "state": {{ "booking": {{ "id": "{task_id}" }} }} }}'
            resp = self.post_task_request()
            self.assertEqual(200, resp.status_code, resp.content)

        # check that the task request is in the database
        resp = self.client.get(f"/tasks/{task_id}/request")
        self.assertEqual(200, resp.status_code, resp.content)
        self.assertEqual("test", resp.json()["category"])
        self.assertEqual("description", resp.json()["description"])

    def test_fail_with_multiple_errors(self):
        # fails with multiple errors
        with patch.object(tasks_service(), "call") as mock:
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
        with patch.object(tasks_service(), "call") as mock:
            mock.return_value = """{
                "success": false
            }
            """
            resp = self.post_task_request()
            self.assertEqual(400, resp.status_code, resp.content)
