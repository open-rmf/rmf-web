# NOTE: This will eventually replace `gateway.py``
from uuid import uuid4

import websocket
from fastapi import FastAPI

from .app_config import app_config
from .models.rmf_api.task_log_update import TaskEventLogUpdate
from .test import AppFixture, make_task_log, try_until

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
        task_log = make_task_log(uuid4().hex)
        task_log_update = TaskEventLogUpdate.construct(
            type="task_log_update", data=task_log
        )
        self.ws.send(task_log_update.json())
        resp = try_until(
            lambda: self.session.get(f"/tasks/{task_log.task_id}/log"),
            lambda x: x.status_code == 200,
        )
        self.assertEqual(200, resp.status_code)
        self.assertEqual(task_log.task_id, resp.json()["task_id"])
