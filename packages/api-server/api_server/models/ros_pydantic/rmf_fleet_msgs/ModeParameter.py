# This is a generated file, do not edit

from typing import List

import pydantic


class ModeParameter(pydantic.BaseModel):
    name: str
    value: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        name: str = "",  # string
        value: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            name=name,
            value=value,
            **kwargs,
        )


# string name
# string value
