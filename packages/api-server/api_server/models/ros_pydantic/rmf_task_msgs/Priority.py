# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Priority(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    value: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]  # uint64


# uint64 value 0
