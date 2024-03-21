# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.Location import Location
from ..rmf_fleet_msgs.RobotMode import RobotMode


class RobotState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str  # string
    model: str  # string
    task_id: str  # string
    seq: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]  # uint64
    mode: RobotMode  # rmf_fleet_msgs/RobotMode
    battery_percent: float  # float32
    location: Location  # rmf_fleet_msgs/Location
    path: list[Location]  # rmf_fleet_msgs/Location


# string name
# string model
#
# # task_id is copied in from the most recent Request message,
# # such as ModeRequest, DestinationRequest, or PathRequest
# string task_id
#
# # The sequence number of this message. Every new message should increment the
# # sequence number by 1.
# uint64 seq
#
# RobotMode mode
# float32 battery_percent
# Location location
# Location[] path
