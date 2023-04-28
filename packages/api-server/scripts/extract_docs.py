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
    ("pnpm", "start"), cwd=f"{os.path.dirname(__file__)}/..", start_new_session=True
)

time.sleep(5)  # wait for server to be ready
outdir = f"{args.output}/docs/api-server/"
os.makedirs(outdir, exist_ok=True)

with urlopen("http://127.0.0.1:8000/docs") as resp:
    html: bytes = resp.read()
    html = html.replace(b"/openapi.json", b"/rmf-web/docs/api-server/openapi.json")
    with open(f"{outdir}/index.html", "bw") as f:
        f.write(html)

with urlopen("http://127.0.0.1:8000/openapi.json") as resp:
    openapi = json.loads(resp.read())
    openapi["servers"] = [
        {
            "url": "{url}",
            "variables": {"url": {"default": "http://example.com"}},
        }
    ]
    with open(f"{outdir}/openapi.json", "w") as f:
        json.dump(openapi, f)
