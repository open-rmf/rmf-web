# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .Location import Location as rmf_fleet_msgs_msg_Location


class DockParameter(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    start: str
    finish: str
    path: list[rmf_fleet_msgs_msg_Location]
