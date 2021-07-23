# This is a generated file, do not edit

from typing import List

import pydantic


class Duration(pydantic.BaseModel):
    sec: pydantic.conint(ge=-2147483648, le=2147483647) = 0  # int32
    nanosec: pydantic.conint(ge=0, le=4294967295) = 0  # uint32

    class Config:
        orm_mode = True


# # Duration defines a period between two time points. It is comprised of a
# # seconds component and a nanoseconds component.
#
# # Seconds component, range is valid over any possible int32 value.
# int32 sec
#
# # Nanoseconds component in the range of [0, 10e9).
# uint32 nanosec
