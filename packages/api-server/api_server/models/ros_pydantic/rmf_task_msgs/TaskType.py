# This is a generated file, do not edit

from typing import Annotated

import pydantic


class TaskType(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    type: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32


# uint32 type
# uint32 TYPE_STATION=0
# uint32 TYPE_LOOP=1
# uint32 TYPE_DELIVERY=2
# uint32 TYPE_CHARGE_BATTERY=3
# uint32 TYPE_CLEAN=4
# uint32 TYPE_PATROL=5
#
