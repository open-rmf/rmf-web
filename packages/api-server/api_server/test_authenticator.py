import unittest
from unittest.mock import AsyncMock, patch

from api_server.authenticator import AuthenticationError, JwtAuthenticator
from api_server.models import User


def _make_authenticator(
    *, preferred_username_claim_namespace: str | None = None
) -> JwtAuthenticator:
    return JwtAuthenticator(
        "test-secret",
        aud="rmf_api_server",
        iss="test",
        preferred_username_claim_namespace=preferred_username_claim_namespace,
    )


class TestGetUser(unittest.IsolatedAsyncioTestCase):
    async def test_bare_preferred_username_claim_is_accepted(self):
        authenticator = _make_authenticator()
        with patch.object(
            User,
            "load_or_create_from_db",
            new=AsyncMock(return_value=User(username="alice")),
        ) as mock_load:
            user = await authenticator._get_user({"preferred_username": "alice"})
        self.assertEqual(user.username, "alice")
        mock_load.assert_awaited_once_with("alice")

    async def test_missing_claim_raises_authentication_error(self):
        authenticator = _make_authenticator()
        with self.assertRaises(AuthenticationError):
            await authenticator._get_user({})

    async def test_namespaced_claim_used_as_fallback_when_namespace_configured(self):
        authenticator = _make_authenticator(
            preferred_username_claim_namespace="https://example.com/"
        )
        claims = {"https://example.com/preferred_username": "bob"}
        with patch.object(
            User,
            "load_or_create_from_db",
            new=AsyncMock(return_value=User(username="bob")),
        ) as mock_load:
            user = await authenticator._get_user(claims)
        self.assertEqual(user.username, "bob")
        mock_load.assert_awaited_once_with("bob")

    async def test_namespaced_claim_ignored_when_namespace_not_configured(self):
        authenticator = _make_authenticator()
        claims = {"https://example.com/preferred_username": "bob"}
        with self.assertRaises(AuthenticationError):
            await authenticator._get_user(claims)

    async def test_bare_claim_preferred_over_namespaced_claim(self):
        authenticator = _make_authenticator(
            preferred_username_claim_namespace="https://example.com/"
        )
        claims = {
            "preferred_username": "alice",
            "https://example.com/preferred_username": "bob",
        }
        with patch.object(
            User,
            "load_or_create_from_db",
            new=AsyncMock(return_value=User(username="alice")),
        ) as mock_load:
            user = await authenticator._get_user(claims)
        self.assertEqual(user.username, "alice")
        mock_load.assert_awaited_once_with("alice")


if __name__ == "__main__":
    unittest.main()
