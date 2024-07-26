# This is a generated file, do not edit

from typing import Annotated

import pydantic


class AlertParameter(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    value: str
