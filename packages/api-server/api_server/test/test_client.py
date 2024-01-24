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
    def __init__(self):
        super().__init__(app)
        self.current_user: str
        self.set_user("admin")

    @classmethod
    def token(cls, username: str) -> str:
        return _generate_token(username)

    def set_user(self, user: str):
        self.current_user = user
        self.headers["Authorization"] = f"bearer {self.token(user)}"
