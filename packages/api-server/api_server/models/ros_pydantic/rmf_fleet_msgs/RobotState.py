# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.Location import Location
from ..rmf_fleet_msgs.RobotMode import RobotMode


class RobotState(pydantic.BaseModel):
    name: str = ""  # string
    model: str = ""  # string
    task_id: str = ""  # string
    seq: pydantic.conint(ge=0, le=18446744073709551615) = 0  # uint64
    mode: RobotMode = RobotMode()  # rmf_fleet_msgs/RobotMode
    battery_percent: float = 0  # float32
    location: Location = Location()  # rmf_fleet_msgs/Location
    path: List[Location] = []  # rmf_fleet_msgs/Location

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "name",
                "model",
                "task_id",
                "seq",
                "mode",
                "battery_percent",
                "location",
                "path",
            ],
        }


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
