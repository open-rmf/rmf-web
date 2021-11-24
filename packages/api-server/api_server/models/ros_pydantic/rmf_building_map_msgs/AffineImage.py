# This is a generated file, do not edit

from typing import List

import pydantic


class AffineImage(pydantic.BaseModel):
    name: str = ""  # string
    x_offset: float = 0  # float32
    y_offset: float = 0  # float32
    yaw: float = 0  # float32
    scale: float = 0  # float32
    encoding: str = ""  # string
    data: bytes = bytes()  # uint8

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "name",
                "x_offset",
                "y_offset",
                "yaw",
                "scale",
                "encoding",
                "data",
            ],
        }


# string name
# float32 x_offset
# float32 y_offset
# float32 yaw
# float32 scale
# string encoding
# uint8[] data
