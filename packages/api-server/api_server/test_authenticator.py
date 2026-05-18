import unittest
from unittest.mock import AsyncMock, patch

import jwt

from api_server.authenticator import AuthenticationError, JwtAuthenticator
from api_server.models import User


SECRET = "test-secret"
AUD = "rmf_api_server"
ISS = "test"


def _make_authenticator(
    *, preferred_username_claim_namespace: str | None = None
) -> JwtAuthenticator:
    return JwtAuthenticator(
        SECRET,
        aud=AUD,
        iss=ISS,
        preferred_username_claim_namespace=preferred_username_claim_namespace,
    )


def _make_token(extra_claims: dict) -> str:
    """Encode a HS256 JWT with the test secret + standard aud/iss + extra claims."""
    claims = {"aud": AUD, "iss": ISS, **extra_claims}
    return jwt.encode(claims, SECRET, algorithm="HS256")


class TestVerifyToken(unittest.IsolatedAsyncioTestCase):
    async def test_bare_preferred_username_claim_is_accepted(self):
        authenticator = _make_authenticator()
        token = _make_token({"preferred_username": "alice"})
        with patch.object(
            User,
            "load_or_create_from_db",
            new=AsyncMock(return_value=User(username="alice")),
        ) as mock_load:
            user = await authenticator.verify_token(token)
        self.assertEqual(user.username, "alice")
        mock_load.assert_awaited_once_with("alice")

    async def test_missing_claim_raises_authentication_error(self):
        authenticator = _make_authenticator()
        token = _make_token({})
        with self.assertRaises(AuthenticationError):
            await authenticator.verify_token(token)

    async def test_namespaced_claim_used_as_fallback_when_namespace_configured(self):
        authenticator = _make_authenticator(
            preferred_username_claim_namespace="https://example.com/"
        )
        token = _make_token({"https://example.com/preferred_username": "bob"})
        with patch.object(
            User,
            "load_or_create_from_db",
            new=AsyncMock(return_value=User(username="bob")),
        ) as mock_load:
            user = await authenticator.verify_token(token)
        self.assertEqual(user.username, "bob")
        mock_load.assert_awaited_once_with("bob")

    async def test_namespaced_claim_ignored_when_namespace_not_configured(self):
        authenticator = _make_authenticator()
        token = _make_token({"https://example.com/preferred_username": "bob"})
        with self.assertRaises(AuthenticationError):
            await authenticator.verify_token(token)

    async def test_bare_claim_preferred_over_namespaced_claim(self):
        authenticator = _make_authenticator(
            preferred_username_claim_namespace="https://example.com/"
        )
        token = _make_token(
            {
                "preferred_username": "alice",
                "https://example.com/preferred_username": "bob",
            }
        )
        with patch.object(
            User,
            "load_or_create_from_db",
            new=AsyncMock(return_value=User(username="alice")),
        ) as mock_load:
            user = await authenticator.verify_token(token)
        self.assertEqual(user.username, "alice")
        mock_load.assert_awaited_once_with("alice")


if __name__ == "__main__":
    unittest.main()
