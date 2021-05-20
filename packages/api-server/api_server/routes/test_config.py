import os

from api_server.default_config import config

test_port = os.environ.get("RMF_SERVER_TEST_PORT", "8000")
config.update({"port": int(test_port), "log_level": "CRITICAL"})
