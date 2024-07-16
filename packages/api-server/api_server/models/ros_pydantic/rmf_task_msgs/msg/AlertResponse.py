# This is a generated file, do not edit

from typing import Annotated

import pydantic


class AlertResponse(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    id: str
    response: str
