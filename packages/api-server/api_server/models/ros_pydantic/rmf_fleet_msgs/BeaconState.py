# This is a generated file, do not edit

from typing import List

import pydantic


class BeaconState(pydantic.BaseModel):
    id: str = ""  # string
    online: bool = False  # bool
    category: str = ""  # string
    activated: bool = False  # bool
    level: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "id",
                "online",
                "category",
                "activated",
                "level",
            ],
        }


# # This message defines data from a robot beacon
#
# string id
# bool online
# string category
# bool activated
# string level
