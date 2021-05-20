# This is a generated file, do not edit

from typing import List

import pydantic


class ModeParameter(pydantic.BaseModel):
    name: str = ""  # string
    value: str = ""  # string

    class Config:
        orm_mode = True


# string name
# string value
