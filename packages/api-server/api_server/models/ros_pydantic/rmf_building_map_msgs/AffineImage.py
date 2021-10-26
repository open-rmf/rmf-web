# This is a generated file, do not edit

from typing import List

import pydantic


class AffineImage(pydantic.BaseModel):
    name: str
    x_offset: float
    y_offset: float
    yaw: float
    scale: float
    encoding: str
    data: bytes

    class Config:
        orm_mode = True

    def __init__(
        self,
        name: str = "",  # string
        x_offset: float = 0,  # float32
        y_offset: float = 0,  # float32
        yaw: float = 0,  # float32
        scale: float = 0,  # float32
        encoding: str = "",  # string
        data: List = bytes(),  # uint8
        **kwargs,
    ):
        super().__init__(
            name=name,
            x_offset=x_offset,
            y_offset=y_offset,
            yaw=yaw,
            scale=scale,
            encoding=encoding,
            data=data or [],
            **kwargs,
        )


# string name
# float32 x_offset
# float32 y_offset
# float32 yaw
# float32 scale
# string encoding
# uint8[] data
