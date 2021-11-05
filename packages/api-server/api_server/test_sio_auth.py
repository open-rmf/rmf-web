import os
import time
import unittest
from typing import Optional

import socketio

from .app import App
from .app_config import load_config
from .test.server import BackgroundServer
from .test.test_fixtures import generate_token


class TestSioAuth(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        app = App(
            app_config=load_config(f"{os.path.dirname(__file__)}/test/test_config.py")
        )
        cls.server = BackgroundServer(app)
        cls.server.start()
        cls.base_url = cls.server.base_url

    @classmethod
    def tearDownClass(cls):
        cls.server.stop()

    def try_connect(self, token: Optional[str] = None) -> bool:
        client = socketio.Client(reconnection=False)
        if token:
            auth = {"token": token}
        else:
            auth = None
        tries = 0
        try:
            while not client.connected:
                try:
                    tries += 1
                    client.connect(self.base_url, auth=auth)
                    return True
                except socketio.exceptions.ConnectionError as e:
                    # Check specifically for connection refused "Errno 111"
                    # Likely a race condition, wait for server to start
                    if "Errno 111" in str(e) and tries < 10:
                        time.sleep(0.5)
                    else:
                        return False
            return False
        finally:
            client.disconnect()

    def test_fail_with_no_token(self):
        self.assertFalse(self.try_connect())

    def test_fail_with_invalid_token(self):
        self.assertFalse(self.try_connect("invalid"))

    def test_success_with_valid_token(self):
        token = generate_token("test_user")
        self.assertTrue(self.try_connect(token))
