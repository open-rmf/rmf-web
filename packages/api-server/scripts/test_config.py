import os

from api_server.default_config import config

here = os.path.dirname(__file__)

test_port = os.environ.get("RMF_API_SERVER_TEST_PORT", "8000")
config.update(
    {
        "host": "127.0.0.1",
        "port": int(test_port),
        "log_level": "ERROR",
        "jwt_public_key": f"{here}/test.pub",
        "jwt_secret": None,
        "iss": "test",
        "db_url": os.environ.get("RMF_API_SERVER_TEST_DB_URL", "sqlite://:memory:"),
        "timezone": "Asia/Singapore",
    }
)
