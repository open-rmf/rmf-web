# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Param(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    type: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
    value_int: Annotated[int, pydantic.Field(ge=-2147483648, le=2147483647)]
    value_float: float
    value_string: str
    value_bool: bool
