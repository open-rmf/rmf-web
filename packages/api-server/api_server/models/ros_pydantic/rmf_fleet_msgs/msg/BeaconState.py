# This is a generated file, do not edit

from typing import Annotated

import pydantic


class BeaconState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    id: str
    online: bool
    category: str
    activated: bool
    level: str
