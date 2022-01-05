# NOTE: This will eventually replace `gateway.py``
from concurrent.futures import Future
from uuid import uuid4

import rx.operators as rxops
import websocket
from fastapi import FastAPI

from .app_config import app_config
from .models import TaskEventLog
from .models.rmf_api.task_log_update import TaskEventLogUpdate
from .rmf_io import task_events
from .test import AppFixture, make_task_log

app = FastAPI()


class TestRmfGatewayApp(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.ws = websocket.WebSocket()
        cls.ws.connect(f"ws://{app_config.host}:{app_config.base_port + 1}")

    @classmethod
    def tearDownClass(cls):
        cls.ws.close()
        super().tearDownClass()

    def test_task_log_update(self):
        task_id = f"task_{uuid4()}"
        task_log = make_task_log(task_id)
        task_log_update = TaskEventLogUpdate(type="task_log_update", data=task_log)
        fut = Future()
        task_events.task_event_logs.subscribe(fut.set_result)
        self.ws.send(task_log_update.json())

        result: TaskEventLog = fut.result(1)
        self.assertEqual(task_id, result.task_id)
        resp = self.session.get(f"/tasks/{task_id}/log?between=0,1636388414500")
        self.assertEqual(200, resp.status_code)
        self.assertEqual(task_id, resp.json()["task_id"])

    def test_task_log_update_can_receive_duplicate_logs(self):
        """
        Logs come in as partial updates, so it is hard to find any duplicates in them.
        The expected behavior is for saving duplicated logs to fail silently, but still
        publishes all received updates, including duplicated ones.
        """
        task_id = f"task_{uuid4()}"
        task_log = make_task_log(task_id)
        task_log_update = TaskEventLogUpdate(type="task_log_update", data=task_log)

        # send same log twice
        for _ in range(2):
            fut = Future()
            task_events.task_event_logs.pipe(rxops.take(1)).subscribe(fut.set_result)
            self.ws.send(task_log_update.json())
            fut.result(1)
