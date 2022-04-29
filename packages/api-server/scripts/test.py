import os

os.environ[
    "RMF_API_SERVER_CONFIG"
] = f"{os.path.dirname(__file__)}/sqlite_test_config.py"

import unittest
from unittest.mock import AsyncMock

from api_server.app import app
from api_server.test import test_client

app.sio = AsyncMock()
result = unittest.main(module=None, exit=False)
test_client.shutdown()
exit(1 if not result.result.wasSuccessful() else 0)
