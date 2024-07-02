import asyncio
import contextlib
import inspect
import os
import os.path
import time
import unittest
import unittest.mock
from collections.abc import Generator
from typing import Awaitable, Callable, Optional, TypeVar, Union
from uuid import uuid4

import pydantic
from anyio.abc import BlockingPortal
from tortoise import Tortoise

from api_server.app import app, app_config
from api_server.models import User
from api_server.routes.admin import PostUsers

from .mocks import patch_sio
from .test_client import TestClient

T = TypeVar("T")


def try_until(
    action: Callable[[], T],
    predicate: Callable[[T], bool],
    timeout=5,
    interval=0.5,
) -> T:
    """
    Do action until an expected result is received.
    Returns the last result.
    """
    end_time = time.time() + timeout

    result = action()
    success = predicate(result)
    if success:
        return result

    time.sleep(interval)
    while time.time() < end_time:
        try:
            result = action()
            success = predicate(result)
            if success:
                return result
        except Exception:  # pylint: disable=broad-except
            pass
        time.sleep(interval)
    return result


async def async_try_until(
    action: Callable[[], Awaitable[T]],
    predicate: Union[Callable[[T], Awaitable[bool]], Callable[[T], bool]],
    timeout=5,
    interval=0.5,
) -> T:
    """
    Do action until an expected result is received.
    Returns the last result, or throws if the last result raises an exception.
    """
    end_time = time.time() + timeout
    while time.time() < end_time:
        try:
            result = await action()
            success = predicate(result)
            if inspect.isawaitable(success):
                success = await success
            if success:
                return result
        except Exception:  # pylint: disable=broad-except
            pass
        await asyncio.sleep(interval)
    return await action()


here = os.path.dirname(__file__)
with open(f"{here}/../../scripts/test.key", "br") as f:
    jwt_key = f.read()


class AppFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        async def clean_db():
            # connect to the db to drop it
            await Tortoise.init(db_url=app_config.db_url, modules={"models": []})
            await Tortoise._drop_databases()  # pylint: disable=protected-access
            # connect to it again to recreate it
            await Tortoise.init(
                db_url=app_config.db_url, modules={"models": []}, _create_db=True
            )
            await Tortoise.close_connections()

        asyncio.run(clean_db())

        cls.admin_user = User(username="admin", is_admin=True)
        cls.client = TestClient()
        cls.client.headers["Content-Type"] = "application/json"
        cls.client.__enter__()
        cls.addClassCleanup(cls.client.__exit__)

    @classmethod
    def get_portal(cls) -> BlockingPortal:
        if not cls.client.portal:
            raise AssertionError(
                "missing client portal, is the client context entered?"
            )
        return cls.client.portal

    @contextlib.contextmanager
    def subscribe_sio(self, room: str, *, user="admin"):
        """
        Subscribes to a socketio room and return a generator of messages
        Returns a tuple of (success: bool, messages: Any).
        """
        if self.client.portal is None:
            raise AssertionError(
                "self.client.portal is None, make sure this is called within a test context"
            )
        portal = self.client.portal

        on_sio_connect = app.sio.handlers["/"]["connect"]
        on_subscribe = app.sio.handlers["/"]["subscribe"]
        on_disconnect = app.sio.handlers["/"]["disconnect"]

        def gen() -> Generator[dict, None, None]:
            async def wait_for_msgs():
                async with condition:
                    if len(msgs) == 0:
                        await condition.wait()
                    return msgs.pop(0)

            while True:
                yield portal.call(asyncio.wait_for, wait_for_msgs(), 5)

        with patch_sio() as mock_sio:
            connected = False
            try:
                msgs = []
                condition = asyncio.Condition()

                async def handle_resp(emit_room, msg, *_args, **_kwargs):
                    if emit_room == "subscribe" and not msg["success"]:
                        # FIXME
                        # pylint: disable=broad-exception-raised
                        raise Exception("Failed to subscribe")
                    if emit_room == room:
                        async with condition:
                            if isinstance(msg, pydantic.BaseModel):
                                msg = msg.model_dump(round_trip=True)
                            msgs.append(msg)
                            condition.notify()

                mock_sio.emit.side_effect = handle_resp

                portal.call(
                    on_sio_connect, "test", {}, {"token": self.client.token(user)}
                )
                connected = True
                portal.call(on_subscribe, "test", {"room": room})

                yield gen()
            finally:
                if connected:
                    portal.call(on_disconnect, "test")

    def setUp(self):
        self.test_time = 0

    def create_user(self, admin: bool = False):
        username = f"user_{uuid4().hex}"
        user = PostUsers(username=username, is_admin=admin)
        resp = self.client.post(
            "/admin/users",
            content=user.model_dump_json(),
        )
        self.assertEqual(200, resp.status_code)
        return username

    def create_role(self):
        role_name = f"role_{uuid4().hex}"
        resp = self.client.post("/admin/roles", json={"name": role_name})
        self.assertEqual(200, resp.status_code)
        return role_name

    def add_permission(self, role: str, action: str, authz_grp: Optional[str] = ""):
        resp = self.client.post(
            f"/admin/roles/{role}/permissions",
            json={"action": action, "authz_grp": authz_grp},
        )
        self.assertEqual(200, resp.status_code)

    def assign_role(self, username: str, role: str):
        resp = self.client.post(f"/admin/users/{username}/roles", json={"name": role})
        self.assertEqual(200, resp.status_code)

    def now(self) -> int:
        """
        Returns the current time in the testing clock in unix millis.
        """
        return self.test_time

    def post_favorite_task(self):
        body = {
            "id": "",
            "name": "",
            "unix_millis_earliest_start_time": 1636388410000,
            "priority": {"type": "binary", "value": 0},
            "category": "clean",
            "description": {"type": "", "zone": ""},
            "user": "",
        }
        return self.client.post(
            "/favorite_tasks",
            json=body,
        )

    def create_favorite_task(self):
        resp = self.post_favorite_task()
        return resp
