# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .Dock import Dock as rmf_fleet_msgs_msg_Dock


class DockSummary(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    docks: list[rmf_fleet_msgs_msg_Dock]
