import os
import shutil
from os.path import join as pathjoin

import tortoise
from api_server.models import tortoise_models as models
from tortoise.contrib.pydantic import pydantic_model_creator

here = os.path.dirname(__file__)
schema_dir = pathjoin(here, "build", "schema")
shutil.rmtree(schema_dir, ignore_errors=True)
os.makedirs(schema_dir, exist_ok=True)


def write_schema(TortoiseModel: tortoise.Model, file_path: str):
    with open(file_path, "w") as f:
        model = pydantic_model_creator(TortoiseModel)
        f.write(model.schema_json(indent=2))
        print(file_path)


write_schema(models.DoorState, pathjoin(schema_dir, "door-state.json"))
write_schema(models.DoorHealth, pathjoin(schema_dir, "door-health.json"))
write_schema(models.LiftState, pathjoin(schema_dir, "lift-state.json"))
write_schema(models.LiftHealth, pathjoin(schema_dir, "lift-health.json"))
write_schema(models.DispenserState, pathjoin(schema_dir, "dispenser-state.json"))
write_schema(models.DispenserHealth, pathjoin(schema_dir, "dispenser-health.json"))
write_schema(models.IngestorState, pathjoin(schema_dir, "ingestor-state.json"))
write_schema(models.IngestorHealth, pathjoin(schema_dir, "ingestor-health.json"))
write_schema(models.FleetState, pathjoin(schema_dir, "fleet-state.json"))
write_schema(models.RobotHealth, pathjoin(schema_dir, "robot-health.json"))
write_schema(models.TaskSummary, pathjoin(schema_dir, "task-summary.json"))
