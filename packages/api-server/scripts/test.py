import unittest

from api_server.test.setup import setup, teardown

setup()
result = unittest.main(module=None, exit=False)
teardown()
exit(1 if not result.result.wasSuccessful() else 0)
