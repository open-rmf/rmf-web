import asyncio
import inspect
import os
import os.path
import time
import unittest
import unittest.mock
from concurrent.futures import Future
from typing import Any, Awaitable, Callable, List, Optional, Tuple, TypeVar, Union
from unittest.mock import patch
from uuid import uuid4

import jwt

from api_server.app import app
from api_server.app import on_connect as on_sio_connect
from api_server.test import client

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
        cls.client = client()
        cls.client.set_user("admin")

    def subscribe_sio(self, room: str, *, user="admin"):
        """
        Subscribes to a socketio room and return a generator of messages
        Returns a tuple of (success: bool, messages: Any).
        """

        def impl():
            with patch.object(app.sio, "emit") as mock:
                loop = asyncio.get_event_loop()
                session = {}
                app.sio.get_session.return_value = session

                fut = asyncio.Future()

                async def wait(emit_room, msg, *_args, **_kwargs):
                    if emit_room == "subscribe" and not msg["success"]:
                        raise Exception("Failed to subscribe")
                    elif emit_room == room:
                        fut.set_result(msg)

                mock.side_effect = wait

                loop.run_until_complete(
                    on_sio_connect("test", {}, {"token": self.client.token(user)})
                )
                loop.run_until_complete(app._on_subscribe("test", {"room": room}))

                yield

                try:
                    yield fut.result()
                    fut = asyncio.Future()
                except asyncio.InvalidStateError:
                    pass

                while True:
                    loop.run_until_complete(asyncio.wait_for(fut, 5))
                    result = fut.result()
                    fut = asyncio.Future()

                    yield result

        gen = impl()
        next(gen)
        return gen

    def setUp(self):
        self.test_time = 0

    def create_user(self, admin: bool = False):
        username = f"user_{uuid4().hex}"
        resp = self.client.post(
            "/admin/users",
            json={"username": username, "is_admin": admin},
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
