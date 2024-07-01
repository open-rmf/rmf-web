# This is a generated file, do not edit

from typing import List

import pydantic


class AlertParameter(pydantic.BaseModel):
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


# # Generic key-value pair to be used in Alert
# string name
# string value
