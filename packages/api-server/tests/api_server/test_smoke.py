import unittest
import os
import subprocess
import time


class TestApiServerSmoke(unittest.TestCase):
    def test_smoke(self):
        if 'TEST_COVERAGE' in os.environ:
            proc = subprocess.Popen(['python3', '-m', 'coverage', 'run', '-a', '-m', 'api_server'])
        else:
            proc = subprocess.Popen(['rmf_api_server'])
        time.sleep(1)
        self.assertIsNone(proc.poll())
        proc.terminate()
        proc.wait(1)
        self.assertEqual(proc.returncode, 0)
        if proc.poll() is not None:
            proc.kill()
            proc.wait()
