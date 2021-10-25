# This is a generated file, do not edit

from typing import List

import pydantic


class Param(pydantic.BaseModel):
    name: str
    type: pydantic.conint(ge=0, le=4294967295)
    value_int: pydantic.conint(ge=-2147483648, le=2147483647)
    value_float: float
    value_string: str
    value_bool: bool

    class Config:
        orm_mode = True

    def __init__(
        self,
        name: str = "",  # string
        type: pydantic.conint(ge=0, le=4294967295) = 0,  # uint32
        value_int: pydantic.conint(ge=-2147483648, le=2147483647) = 0,  # int32
        value_float: float = 0,  # float32
        value_string: str = "",  # string
        value_bool: bool = False,  # bool
    ):
        super().__init__(
            name=name,
            type=type,
            value_int=value_int,
            value_float=value_float,
            value_string=value_string,
            value_bool=value_bool,
        )


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
