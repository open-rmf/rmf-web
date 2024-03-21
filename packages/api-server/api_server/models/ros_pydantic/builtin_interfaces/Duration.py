# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Duration(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    sec: Annotated[int, pydantic.Field(ge=-2147483648, le=2147483647)]  # int32
    nanosec: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32


# # Duration defines a period between two time points.
# # Messages of this datatype are of ROS Time following this design:
# # https://design.ros2.org/articles/clock_and_time.html
#
# # Seconds component, range is valid over any possible int32 value.
# int32 sec
#
# # Nanoseconds component in the range of [0, 10e9).
# uint32 nanosec
