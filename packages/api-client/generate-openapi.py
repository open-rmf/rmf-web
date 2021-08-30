import json
import os

from api_server.app import App

here = os.path.realpath(os.path.dirname(__file__))
os.makedirs(f"{here}/build", exist_ok=True)
with open(f"{here}/build/openapi.json", "w") as f:
    json.dump(App().fapi.openapi(), f)
