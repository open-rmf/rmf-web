import asyncio
from unittest.mock import AsyncMock, patch

from api_server.app import app, on_sio_connect
from api_server.authenticator import authenticator

from .test.test_fixtures import AppFixture


class TestSioAuth(AppFixture):
    def test_token_is_verified(self):
        with patch.object(
            authenticator, "verify_token"
        ) as mock_verify_token, patch.object(app, "sio") as mock_sio:
            # set up mocks
            session = {}
            mock_sio.get_session = AsyncMock(return_value=session)

            asyncio.run(on_sio_connect("test", {}, {"token": "test-token"}))
            mock_verify_token.assert_awaited_once_with("test-token")
