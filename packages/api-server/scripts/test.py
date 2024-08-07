import os

os.environ["RMF_API_SERVER_CONFIG"] = f"{os.path.dirname(__file__)}/test_config.py"

import unittest

result = unittest.main(module=None, exit=False)
exit(1 if not result.result.wasSuccessful() else 0)
