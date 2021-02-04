import unittest
import subprocess
import time

from api_server.__main__ import main


class TestApiServerSmoke(unittest.TestCase):
    def test_smoke(self):
        proc = subprocess.Popen(['python3', '-m', 'coverage', 'run', '-m', 'api_server'])
        time.sleep(1)
        self.assertIsNone(proc.poll())
        proc.terminate()
        proc.wait(1)
        self.assertEqual(proc.returncode, 0)
        if proc.poll() is not None:
            proc.kill()
            proc.wait()
