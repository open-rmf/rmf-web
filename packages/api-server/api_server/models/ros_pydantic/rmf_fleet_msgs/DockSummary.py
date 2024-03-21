# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.Dock import Dock


class DockSummary(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    docks: list[Dock]  # rmf_fleet_msgs/Dock


# Dock[] docks
