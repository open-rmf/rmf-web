# This is a generated file, do not edit

from typing import Annotated

import pydantic


class DispatchAck(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    dispatch_id: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
    success: bool
    errors: list[str]
