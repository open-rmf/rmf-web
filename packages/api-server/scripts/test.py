import unittest

from api_server.test.test_server import test_server

test_server.run_in_background()
result = unittest.main(module=None, exit=False)
test_server.shutdown()
exit(1 if not result.result.wasSuccessful() else 0)
