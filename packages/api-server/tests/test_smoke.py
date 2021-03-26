import os
import time
import unittest

from .server import start_server


class TestApiServerSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = start_server(f"{os.path.dirname(__file__)}/config.py")

    @classmethod
    def tearDownClass(cls):
        cls.server.kill()
        cls.server.wait()

    def test_smoke(self):
        time.sleep(5)
        self.assertIsNone(self.server.poll())
        self.server.terminate()
        self.server.wait(5)
        self.assertEqual(self.server.returncode, 0)
