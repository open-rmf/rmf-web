import os
import subprocess


def start_server(config: str):
    env = os.environ.copy()
    env["RMF_API_SERVER_CONFIG"] = config
    if "TEST_COVERAGE" in os.environ:
        proc = subprocess.Popen(
            ["python", "-m", "coverage", "run", "-a", "-m", "api_server"], env=env
        )
    else:
        proc = subprocess.Popen(["rmf_api_server"], env=env)
    return proc
