# This is a generated file, do not edit

from typing import Annotated

import pydantic


class MutexGroupManualRelease(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    release_mutex_groups: list[str]
    fleet: str
    robot: str
