# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .Location import Location as rmf_fleet_msgs_msg_Location
from .RobotMode import RobotMode as rmf_fleet_msgs_msg_RobotMode


class RobotState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    model: str
    task_id: str
    seq: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
    mode: rmf_fleet_msgs_msg_RobotMode
    battery_percent: float
    location: rmf_fleet_msgs_msg_Location
    path: list[rmf_fleet_msgs_msg_Location]
