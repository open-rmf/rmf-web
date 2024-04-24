# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Param(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str  # string
    type: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32
    value_int: Annotated[int, pydantic.Field(ge=-2147483648, le=2147483647)]  # int32
    value_float: float  # float32
    value_string: str  # string
    value_bool: bool  # bool


# string name
#
# uint32 type
# uint32 TYPE_UNDEFINED=0
# uint32 TYPE_STRING=1
# uint32 TYPE_INT=2
# uint32 TYPE_DOUBLE=3
# uint32 TYPE_BOOL=4
#
# int32 value_int
# float32 value_float
# string value_string
# bool value_bool
