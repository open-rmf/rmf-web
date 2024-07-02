# This is a generated file, do not edit

from typing import Annotated

import pydantic


class BeaconState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    id: str  # string
    online: bool  # bool
    category: str  # string
    activated: bool  # bool
    level: str  # string


# # This message defines data from a robot beacon
#
# string id
# bool online
# string category
# bool activated
# string level
