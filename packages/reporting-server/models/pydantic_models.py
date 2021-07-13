from tortoise.contrib.pydantic import pydantic_model_creator

from .tortoise_models import (
    AuthEvents,
    Container,
    DispenserState,
    DoorState,
    FleetState,
    HealthStatus,
    IngestorState,
    LiftState,
    RawLog,
    TaskSummary,
)

DispenserState_Pydantic = pydantic_model_creator(DispenserState, name="DispenserState")

AuthEvents_Pydantic = pydantic_model_creator(AuthEvents, name="AuthEvents")
Container_Pydantic = pydantic_model_creator(Container, name="Container")
DoorState_Pydantic = pydantic_model_creator(DoorState, name="DoorState")
FleetState_Pydantic = pydantic_model_creator(FleetState, name="FleetState")
HealthStatus_Pydantic = pydantic_model_creator(HealthStatus, name="HealthStatus")
IngestorState_Pydantic = pydantic_model_creator(IngestorState, name="IngestorState")
LiftState_Pydantic = pydantic_model_creator(LiftState, name="LiftStates")
RawLog_Pydantic = pydantic_model_creator(RawLog, name="RawLog")
TaskSummary_Pydantic = pydantic_model_creator(TaskSummary, name="TaskSummary")

print(RawLog_Pydantic.schema_json(indent=4))
