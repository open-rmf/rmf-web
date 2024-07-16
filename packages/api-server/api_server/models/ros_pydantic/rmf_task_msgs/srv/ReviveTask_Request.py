# This is a generated file, do not edit

from typing import Annotated

import pydantic


class ReviveTask_Request(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    requester: str
    task_id: str
