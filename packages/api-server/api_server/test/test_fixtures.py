import asyncio
import inspect
import os
import os.path
import time
import unittest
from concurrent.futures import Future
from typing import Awaitable, Callable, List, Optional, TypeVar, Union, cast
from unittest.mock import Mock
from uuid import uuid4

import jwt
import requests
import socketio
from urllib3.util.retry import Retry

from api_server.app import App
from api_server.app_config import load_config
from api_server.gateway import RmfGateway

from .server import BackgroundServer

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
    while time.time() < end_time:
        try:
            result = action()
            success = predicate(result)
        except Exception:  # pylint: disable=broad-except
            success = False
        if success:
            return result
        time.sleep(interval)
    return result


async def async_try_until(
    action: Callable[[], Awaitable[T]],
    predicate: Union[Callable[[T], Awaitable[bool]], Callable[[T], bool]],
    timeout=5,
    interval=0.5,
) -> Awaitable[T]:
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
with open(f"{here}/test.key", "br") as f:
    jwt_key = f.read()


def generate_token(username: str):
    return jwt.encode(
        {
            "aud": "test",
            "iss": "test",
            "preferred_username": username,
        },
        jwt_key,
        "RS256",
    )


def rmf_gateway_fc(rmf_events, static_files_repo):
    orig = RmfGateway(rmf_events, static_files_repo)
    rmf_gateway = Mock(orig)
    rmf_gateway.now = orig.now
    return rmf_gateway


class PrefixUrlSession(requests.Session):
    def __init__(self, prefix_url: str, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.prefix_url = prefix_url

    def request(self, method, url, **kwargs):  # pylint: disable=arguments-differ
        return super().request(method, f"{self.prefix_url}{url}", **kwargs)


class AppFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = App(
            app_config=load_config(f"{os.path.dirname(__file__)}/test_config.py"),
            rmf_gateway_fc=rmf_gateway_fc,
        )
        ready = Future()
        cls.app.fapi.on_event("startup")(lambda: ready.set_result(True))
        cls.server = BackgroundServer(cls.app)
        cls.server.start()

        retry = Retry(total=5, backoff_factor=0.1)
        adapter = requests.adapters.HTTPAdapter(max_retries=retry)
        cls.session = cast(requests.Session, PrefixUrlSession(cls.server.base_url))

        cls.set_user("admin")
        cls.session.headers["Content-Type"] = "application/json"
        cls.session.mount("http://", adapter)

        ready.result()

    @classmethod
    def tearDownClass(cls):
        cls.session.close()
        cls.server.stop()

    def setUp(self):
        self._sioClients: List[socketio.Client] = []

    def tearDown(self):
        for client in self._sioClients:
            if client.connected:
                client.disconnect()

    @classmethod
    def run_in_app_loop(cls, work: Awaitable, timeout: Optional[float] = None):
        fut = Future()

        async def task():
            fut.set_result(await work)

        cls.server.app.loop.create_task(task())
        return fut.result(timeout)

    @classmethod
    def set_user(cls, username: str):
        token = generate_token(username)
        cls.session.headers["Authorization"] = f"bearer {token}"

    def subscribe_sio(self, path: str, skip_first=True):
        client = self.connect_sio()
        fut = Future()
        count = 0
        if skip_first:
            needed = 2
        else:
            needed = 1

        def on_event(data):
            nonlocal count
            count += 1
            if count >= needed:
                client.disconnect()
                fut.set_result(data)

        client.on(path, on_event)
        client.emit("subscribe", {"path": path})

        return fut

    def connect_sio(self, user="admin"):
        sioClient = socketio.Client()
        sioClient.connect(self.server.base_url, auth={"token": generate_token(user)})
        self._sioClients.append(sioClient)
        return sioClient

    def create_user(self, admin: bool = False):
        username = f"user_{uuid4().hex}"
        resp = self.session.post(
            "/admin/users",
            json={"username": username, "is_admin": admin},
        )
        self.assertEqual(200, resp.status_code)
        return username

    def create_role(self):
        role_name = f"role_{uuid4().hex}"
        resp = self.session.post("/admin/roles", json={"name": role_name})
        self.assertEqual(200, resp.status_code)
        return role_name

    def add_permission(self, role: str, action: str, authz_grp: Optional[str] = ""):
        resp = self.session.post(
            f"/admin/roles/{role}/permissions",
            json={"action": action, "authz_grp": authz_grp},
        )
        self.assertEqual(200, resp.status_code)

    def assign_role(self, username: str, role: str):
        resp = self.session.post(f"/admin/users/{username}/roles", json={"name": role})
        self.assertEqual(200, resp.status_code)
