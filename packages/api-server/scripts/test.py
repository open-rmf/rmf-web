import asyncio
import os

from tortoise import Tortoise

os.environ[
    "RMF_API_SERVER_CONFIG"
] = f"{os.path.dirname(__file__)}/sqlite_test_config.py"

import unittest

result = unittest.main(module=None, exit=False)
asyncio.run(Tortoise.close_connections())
exit(1 if not result.result.wasSuccessful() else 0)
