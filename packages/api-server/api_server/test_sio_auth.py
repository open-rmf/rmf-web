import asyncio
from typing import Optional

from api_server.app import on_connect as on_sio_connect

from .test import client
from .test.test_fixtures import AppFixture


class TestSioAuth(AppFixture):
    def try_connect(self, token: Optional[str]):
        loop = asyncio.get_event_loop()
        fut = asyncio.Future()

        async def result():
            fut.set_result(await on_sio_connect("test", {}, {"token": token}))

        loop.run_until_complete(result())
        return fut.result()

    def test_fail_with_no_token(self):
        self.assertFalse(self.try_connect(None))

    def test_fail_with_invalid_token(self):
        self.assertFalse(self.try_connect("invalid"))

    def test_success_with_valid_token(self):
        self.assertTrue(self.try_connect(client().token("admin")))
