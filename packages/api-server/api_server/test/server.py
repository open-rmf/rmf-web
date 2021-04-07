import os
import subprocess
from typing import Optional


def launch_server(config: Optional[str] = None):
    if "RMF_API_SERVER_TEST_NO_LAUNCH" in os.environ and (
        os.environ["RMF_API_SERVER_TEST_NO_LAUNCH"].lower() == "true"
        or os.environ["RMF_API_SERVER_TEST_NO_LAUNCH"] != "0"
    ):
        return None
    env = os.environ.copy()
    if config:
        env["RMF_API_SERVER_CONFIG"] = config
    if "RMF_API_SERVER_TEST_COVERAGE" in os.environ and (
        os.environ["RMF_API_SERVER_TEST_COVERAGE"].lower() == "true"
        or os.environ["RMF_API_SERVER_TEST_COVERAGE"] != "0"
    ):
        proc = subprocess.Popen(
            ["python", "-m", "coverage", "run", "-p", "-m", "api_server"], env=env
        )
    else:
        proc = subprocess.Popen(["python", "-m", "api_server"], env=env)
    return proc


def terminate_server(server_proc: subprocess.Popen):
    try:
        server_proc.terminate()
        server_proc.wait(5)
    except subprocess.TimeoutExpired:
        server_proc.kill()
        server_proc.wait()
