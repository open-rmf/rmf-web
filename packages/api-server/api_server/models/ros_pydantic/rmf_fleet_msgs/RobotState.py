# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.Location import Location
from ..rmf_fleet_msgs.RobotMode import RobotMode


class RobotState(pydantic.BaseModel):
    name: str
    model: str
    task_id: str
    seq: pydantic.conint(ge=0, le=18446744073709551615)
    mode: RobotMode
    battery_percent: float
    location: Location
    path: List[Location]

    class Config:
        orm_mode = True

    def __init__(
        self,
        name: str = "",  # string
        model: str = "",  # string
        task_id: str = "",  # string
        seq: int = 0,  # uint64
        mode: RobotMode = RobotMode(),  # rmf_fleet_msgs/RobotMode
        battery_percent: float = 0,  # float32
        location: Location = Location(),  # rmf_fleet_msgs/Location
        path: List = None,  # rmf_fleet_msgs/Location
        **kwargs,
    ):
        super().__init__(
            name=name,
            model=model,
            task_id=task_id,
            seq=seq,
            mode=mode,
            battery_percent=battery_percent,
            location=location,
            path=path or [],
            **kwargs,
        )


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
