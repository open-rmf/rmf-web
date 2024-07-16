# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .DockParameter import DockParameter as rmf_fleet_msgs_msg_DockParameter


class Dock(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    params: list[rmf_fleet_msgs_msg_DockParameter]
