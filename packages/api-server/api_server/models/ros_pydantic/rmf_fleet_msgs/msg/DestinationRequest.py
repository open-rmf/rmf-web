# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .Location import Location as rmf_fleet_msgs_msg_Location


class DestinationRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    robot_name: str
    destination: rmf_fleet_msgs_msg_Location
    task_id: str
