import argparse
import atexit
import json
import os.path
import signal
import subprocess
import time
from urllib.request import urlopen

parser = argparse.ArgumentParser(
    description="Extracts openapi docs from the api server. NOTE: Make sure the api server is not running before running this."
)
parser.add_argument("-o", "--output", help="output directory", required=True)
args = parser.parse_args()

server_proc = None


def cleanup():
    if server_proc:
        pgid = os.getpgid(server_proc.pid)
        os.killpg(pgid, signal.SIGTERM)
        server_proc.wait()


atexit.register(cleanup)
server_proc = subprocess.Popen(
    ("python", "-m", "api_server"),
    cwd=f"{os.path.dirname(__file__)}/..",
    start_new_session=True,
)

time.sleep(5)  # wait for server to be ready
outdir = f"{args.output}"
os.makedirs(outdir, exist_ok=True)

base_url = "http://localhost:8000/rmf-web"
with urlopen(f"{base_url}/docs") as resp:
    html: bytes = resp.read()
    with open(f"{outdir}/index.html", "bw") as f:
        f.write(html)

with urlopen(f"{base_url}/openapi.json") as resp:
    openapi = json.loads(resp.read())
    openapi["servers"] = [
        {
            "url": "{url}",
            "variables": {"url": {"default": "http://example.com"}},
        }
    ]
    with open(f"{outdir}/openapi.json", "w") as f:
        json.dump(openapi, f)

files_to_download = [
    "/static/swagger-ui-bundle.js",
    "/static/swagger-ui.css",
]

for p in files_to_download:
    with urlopen(f"{base_url}{p}") as resp:
        fp = f"{outdir}{p}"
        os.makedirs(os.path.dirname(fp), exist_ok=True)
        with open(fp, "bw") as f:
            f.write(resp.read())
