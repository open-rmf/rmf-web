import asyncio
from typing import Optional
from unittest.mock import AsyncMock, patch

from api_server.app import app, on_sio_connect

from .test import client
from .test.test_fixtures import AppFixture


class TestSioAuth(AppFixture):
    @staticmethod
    def try_connect(token: Optional[str]):
        with patch.object(app, "sio") as mock:
            # set up mocks
            session = {}
            mock.get_session = AsyncMock(return_value=session)

            loop = asyncio.new_event_loop()
            fut = asyncio.Future(loop=loop)

            async def result():
                fut.set_result(await on_sio_connect("test", {}, {"token": token}))

            loop.run_until_complete(result())
            loop.close()
            return fut.result()

    def test_fail_with_no_token(self):
        self.assertFalse(self.try_connect(None))

    def test_fail_with_invalid_token(self):
        self.assertFalse(self.try_connect("invalid"))

    def test_success_with_valid_token(self):
        self.assertTrue(self.try_connect(client().token("admin")))
