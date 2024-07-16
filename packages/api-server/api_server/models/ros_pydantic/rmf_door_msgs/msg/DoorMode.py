# This is a generated file, do not edit

from typing import Annotated

import pydantic


class DoorMode(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    value: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
