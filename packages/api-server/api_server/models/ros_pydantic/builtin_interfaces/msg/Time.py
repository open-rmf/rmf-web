# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Time(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    sec: Annotated[int, pydantic.Field(ge=-2147483648, le=2147483647)]
    nanosec: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
