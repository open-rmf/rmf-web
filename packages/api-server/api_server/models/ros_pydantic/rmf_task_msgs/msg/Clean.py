# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Clean(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    start_waypoint: str
