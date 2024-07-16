# This is a generated file, do not edit

from typing import Annotated

import pydantic


class ApiResponse(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    type: Annotated[int, pydantic.Field(ge=0, le=255)]
    json_msg: str
    request_id: str
