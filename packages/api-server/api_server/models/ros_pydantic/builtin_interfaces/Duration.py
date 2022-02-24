# This is a generated file, do not edit

from typing import List

import pydantic


class Duration(pydantic.BaseModel):
    sec: pydantic.conint(ge=-2147483648, le=2147483647) = 0  # int32
    nanosec: pydantic.conint(ge=0, le=4294967295) = 0  # uint32

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "sec",
                "nanosec",
            ],
        }


# # Duration defines a period between two time points.
# # Messages of this datatype are of ROS Time following this design:
# # https://design.ros2.org/articles/clock_and_time.html
#
# # Seconds component, range is valid over any possible int32 value.
# int32 sec
#
# # Nanoseconds component in the range of [0, 10e9).
# uint32 nanosec
