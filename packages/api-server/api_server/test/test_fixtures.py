import asyncio
import inspect
import os
import os.path
import time
import unittest
from typing import Awaitable, Callable, Optional, TypeVar, Union
from unittest.mock import Mock
from uuid import uuid4

import jwt
from httpx import AsyncClient

from api_server.app import App
from api_server.app_config import load_config
from api_server.gateway import RmfGateway

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
        result = action()
        success = predicate(result)
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


test_app = App(
    app_config=load_config(f"{os.path.dirname(__file__)}/test_config.py"),
    rmf_gateway_fc=rmf_gateway_fc,
)


class AppFixture(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        # def rmf_gateway_fc(rmf_events, static_files_repo):
        #     orig = RmfGateway(rmf_events, static_files_repo)
        #     self.rmf_gateway = Mock(orig)
        #     self.rmf_gateway.now = orig.now
        #     return self.rmf_gateway

        # self.app = App(
        #     app_config=load_config(f"{os.path.dirname(__file__)}/test_config.py"),
        #     rmf_gateway_fc=rmf_gateway_fc,
        # )
        self.app = test_app

        for handler in self.app.fapi.router.on_startup:
            if asyncio.iscoroutinefunction(handler):
                await handler()
            else:
                handler()
        self.client = AsyncClient(app=self.app, base_url="http://test")
        self.set_user("admin")
        await self.client.__aenter__()

    async def asyncTearDown(self):
        await self.client.__aexit__()
        for handler in self.app.fapi.router.on_shutdown:
            if asyncio.iscoroutinefunction(handler):
                await handler()
            else:
                handler()

    def set_user(self, username: str):
        token = generate_token(username)
        self.client.headers["Authorization"] = f"bearer {token}"

    async def create_user(self, admin: bool = False):
        username = f"user_{uuid4().hex}"
        resp = await self.client.post(
            "/admin/users",
            json={"username": username, "is_admin": admin},
        )
        self.assertEqual(200, resp.status_code)
        return username

    async def create_role(self):
        role_name = f"role_{uuid4().hex}"
        resp = await self.client.post("/admin/roles", json={"name": role_name})
        self.assertEqual(200, resp.status_code)
        return role_name

    async def add_permission(
        self, role: str, action: str, authz_grp: Optional[str] = ""
    ):
        resp = await self.client.post(
            f"/admin/roles/{role}/permissions",
            json={"action": action, "authz_grp": authz_grp},
        )
        self.assertEqual(200, resp.status_code)

    async def assign_role(self, username: str, role: str):
        resp = await self.client.post(
            f"/admin/users/{username}/roles", json={"name": role}
        )
        self.assertEqual(200, resp.status_code)
