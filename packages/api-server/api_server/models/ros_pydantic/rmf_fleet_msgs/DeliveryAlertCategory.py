# This is a generated file, do not edit

from typing import Annotated

import pydantic


class DeliveryAlertCategory(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    value: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32


# uint32 value
# uint32 MISSING=0
# uint32 WRONG=1
# uint32 OBSTRUCTED=2
# uint32 CANCELLED=3
