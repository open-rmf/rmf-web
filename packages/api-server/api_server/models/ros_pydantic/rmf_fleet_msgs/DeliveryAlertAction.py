# This is a generated file, do not edit

from typing import Annotated

import pydantic


class DeliveryAlertAction(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    value: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32


# uint32 value
# uint32 WAITING=0
# uint32 CANCEL=1
# uint32 OVERRIDE=2
# uint32 RESUME=3
