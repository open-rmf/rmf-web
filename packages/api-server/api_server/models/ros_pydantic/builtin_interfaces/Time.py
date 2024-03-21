# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Time(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    sec: Annotated[int, pydantic.Field(ge=-2147483648, le=2147483647)]  # int32
    nanosec: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32


# # This message communicates ROS Time defined here:
# # https://design.ros2.org/articles/clock_and_time.html
#
# # The seconds component, valid over all int32 values.
# int32 sec
#
# # The nanoseconds component, valid in the range [0, 10e9).
# uint32 nanosec
