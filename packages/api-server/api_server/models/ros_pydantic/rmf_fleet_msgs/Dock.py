# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.DockParameter import DockParameter


class Dock(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str  # string
    params: list[DockParameter]  # rmf_fleet_msgs/DockParameter


# string fleet_name
# DockParameter[] params
