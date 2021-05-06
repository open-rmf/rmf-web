# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.DockParameter import DockParameter


class Dock(pydantic.BaseModel):
    fleet_name: str = ""  # string
    params: List[DockParameter] = []  # rmf_fleet_msgs/DockParameter


# string fleet_name
# DockParameter[] params
