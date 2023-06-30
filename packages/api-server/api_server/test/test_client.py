import os.path
from typing import Optional

import jwt
from fastapi.testclient import TestClient as BaseTestClient

from api_server.app import app

here = os.path.dirname(__file__)


_jwt_key: Optional[str] = None


def jwt_key():
    global _jwt_key
    if _jwt_key is None:
        with open(f"{here}/../../scripts/test.key", "r", encoding="utf8") as f:
            _jwt_key = f.read()
    return _jwt_key


def _generate_token(username: str):
    return jwt.encode(
        {
            "aud": "rmf_api_server",
            "iss": "test",
            "preferred_username": username,
        },
        jwt_key(),
        "RS256",
    )


class TestClient(BaseTestClient):
    _admin_token: Optional[str] = None

    def __init__(self):
        super().__init__(app)

    @classmethod
    def token(cls, username: str) -> str:
        if username == "admin":
            if cls._admin_token is None:
                cls._admin_token = _generate_token("admin")
            return cls._admin_token

        return _generate_token(username)

    def set_user(self, user):
        self.headers["Authorization"] = f"bearer {self.token(user)}"


_client: Optional[TestClient] = None


def client(user="admin") -> TestClient:
    global _client
    if _client is None:
        _client = TestClient()
        _client.__enter__()
    _client.headers["Content-Type"] = "application/json"
    _client.set_user(user)
    return _client


def shutdown():
    global _client
    if _client is not None:
        _client.__exit__()
