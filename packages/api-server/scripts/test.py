import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("config", choices=["sqlite"], nargs="?")
args = parser.parse_args()

if args.config == "sqlite" or args.config is None:
    os.environ[
        "RMF_API_SERVER_CONFIG"
    ] = f"{os.path.dirname(__file__)}/sqlite_test_config.py"

import unittest

from api_server.test.test_server import test_server

test_server.run_in_background()
result = unittest.main(module=None, exit=False)
test_server.shutdown()
exit(1 if not result.result.wasSuccessful() else 0)
