# This is a generated file, do not edit

from typing import Annotated

import pydantic


class BehaviorParameter(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str  # string
    value: str  # string


# string name
# string value
