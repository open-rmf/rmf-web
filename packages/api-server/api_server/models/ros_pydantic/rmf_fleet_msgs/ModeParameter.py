# This is a generated file, do not edit

from typing import List

import pydantic


class ModeParameter(pydantic.BaseModel):
    name: str = ""  # string
    value: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "name",
                "value",
            ],
        }


# string name
# string value
